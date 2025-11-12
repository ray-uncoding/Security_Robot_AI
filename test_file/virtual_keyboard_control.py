# ====================================================
# ========== Step 1 基本匯入與初始化 =============
# ====================================================

# 1.1 標準庫與第三方套件匯入
import pyaudio
import speech_recognition as sr
import google.generativeai as genai
import json
import os
import time, math, threading

# 1.2 全域變數，決定是否真的控制機器人
CONTROL_ROBOT = True   # True: 發送 /cmd_vel 控制車子；False: 只印出指令

if CONTROL_ROBOT:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, PoseStamped
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# ====================================================
# ========== Step 2 機器人控制 =============
# ====================================================

    # 2.1 機器人控制類別
    class RobotController(Node):
        
        # 2.2 初始化
        def __init__(self):
            super().__init__('voice_command_controller')
            self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
            
            # 初始化 Nav2 導航
            self.navigator = BasicNavigator()
            
            # 載入航點配置
            self.waypoints = self.load_waypoints()

            # 你原本的基準速度（可依需求調）
            self.linear_speed  = 0.20   # m/s
            self.angular_speed = 0.50   # rad/s

            # 內部狀態：讓新命令可以中斷舊的連續發送
            self._stop_event = threading.Event()
            self._worker = None

        # 2.3 連續發送 Twist 的內部函式
        def _publish_twist_stream(self, lin_x: float, ang_z: float, hz: int, duration: float):
            """連續在 hz 頻率下發送 Twist，持續 duration 秒；支援 stop 中斷。"""
            period = 1.0 / float(hz)
            t_end = time.monotonic() + max(0.0, duration)

            twist = Twist()
            while not self._stop_event.is_set() and time.monotonic() < t_end:                   # 持續發送直到時間到或被中斷
                twist.linear.x = lin_x;  twist.linear.y = 0.0; twist.linear.z = 0.0             # 線速度
                twist.angular.x = 0.0;   twist.angular.y = 0.0; twist.angular.z = ang_z         # 角速度
                self.publisher_.publish(twist)                                                  # 發送
                time.sleep(period)

            # 停止：補發 0 讓車確實煞住
            stop = Twist()
            self.publisher_.publish(stop)

        # 2.4 中斷目前的連續發送
        def stop_motion(self):
            """中斷目前的連續發送（等同於放開鍵、或語音說 stop）。"""
            self._stop_event.set()
            if self._worker and self._worker.is_alive():
                self._worker.join(timeout=1.0)
            self._worker = None
            self._stop_event.clear()

        # 2.5 主要的指令轉換與發送函式
        def send_command(self, action: str, value: float = 0.0, hold_sec: float | None = None, hz: int = 20):
            """
            將抽象指令轉為連續 /cmd_vel：
            - move_forward/backward: value 當「距離（公尺）」；若沒給 value 就用 hold_sec 或預設 2 秒
            - turn_left/right: value 當「角度（度）」；若沒給 value 就用 hold_sec 或預設 2 秒
            - stop: 立刻中斷、並補發 0
            """
            # 先中止任何舊的流
            self.stop_motion()

            # 轉成 (lin_x, ang_z) 與 duration
            lin_x, ang_z = 0.0, 0.0
            duration = 2.0 if hold_sec is None else float(hold_sec)

            if action == "move_forward":
                lin_x = +self.linear_speed
                if value and value > 0:
                    duration = max(0.05, float(value) / abs(self.linear_speed))
            elif action == "move_backward":
                lin_x = -self.linear_speed
                if value and value > 0:
                    duration = max(0.05, float(value) / abs(self.linear_speed))
            elif action == "turn_left":
                ang_z = +self.angular_speed
                if value and value != 0:
                    rad = float(value) * math.pi / 180.0
                    duration = max(0.05, abs(rad) / abs(self.angular_speed))
            elif action == "turn_right":
                ang_z = -self.angular_speed
                if value and value != 0:
                    rad = float(value) * math.pi / 180.0
                    duration = max(0.05, abs(rad) / abs(self.angular_speed))
            elif action == "stop":
                # 直接停、補發零
                self._publish_twist_stream(0.0, 0.0, hz=hz, duration=0.05)
                self.get_logger().info("✅ 停止")
                return
            else:
                self.get_logger().warning(f"❌ 未知指令: {action}")
                return

            # 開新 thread 連續發送（等同「按住鍵」）
            self._stop_event.clear()
            self._worker = threading.Thread(
                target=self._publish_twist_stream,
                args=(lin_x, ang_z, hz, duration),
                daemon=True
            )
            self._worker.start()
            self.get_logger().info(f"▶️ 連續發送: action={action}, lin_x={lin_x:.3f}, ang_z={ang_z:.3f}, "
                                f"hz={hz}, duration={duration:.2f}s")

        # 2.6 載入航點配置
        def load_waypoints(self):
            """載入航點配置檔案"""
            try:
                with open('/home/nvidia/workspace/Security_Robot_AI/robot_projects/Sr_robot_Base/wheeltec_robot_nav2/map/saved_points.json', 'r') as file:
                    data = json.load(file)
                waypoints = data["points"]
                self.get_logger().info(f"✅ 成功載入 {len(waypoints)} 個航點")
                return waypoints
            except Exception as e:
                self.get_logger().error(f"❌ 載入航點失敗: {e}")
                return []

        # 2.7 導航到指定點位
        def navigate_to_waypoint(self, point_number):
            """導航到指定編號的點位"""
            try:
                if not self.waypoints:
                    self.get_logger().error("❌ 沒有可用的航點")
                    return False
                    
                if point_number < 1 or point_number > len(self.waypoints):
                    self.get_logger().error(f"❌ 點位編號 {point_number} 超出範圍 (1-{len(self.waypoints)})")
                    return False
                
                # 等待 Nav2 啟動
                self.navigator.waitUntilNav2Active()
                
                # 選擇目標點位 (從 1 開始計數，所以要 -1)
                wp = self.waypoints[point_number - 1]
                
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                goal_pose.pose.position.x = wp["x"]
                goal_pose.pose.position.y = wp["y"]
                goal_pose.pose.position.z = wp["z"]
                goal_pose.pose.orientation.x = wp["qx"]
                goal_pose.pose.orientation.y = wp["qy"]
                goal_pose.pose.orientation.z = wp["qz"]
                goal_pose.pose.orientation.w = wp["qw"]
                
                self.get_logger().info(f"🎯 開始導航到點位 {point_number}: ({wp['x']:.2f}, {wp['y']:.2f})")
                
                # 開始導航
                self.navigator.goToPose(goal_pose)
                
                return True
                
            except Exception as e:
                self.get_logger().error(f"❌ 導航失敗: {e}")
                return False

# ====================================================
# ========== Step 3 Gemini API 初始化 =============
# ====================================================

# 3.1 初始化 Gemini 模型
genai.configure(api_key=os.environ.get("GEMINI_API_KEY", "YOUR_API_KEY"))

# 3.2 Schema 驅動 JSON 輸出格式 - 基本移動
response_schema_basic = {
    "type": "object",
    "properties": {
        "action": {
            "type": "string",
            "enum": ["move_forward", "move_backward", "turn_left", "turn_right", "stop"]
        },
        "value": {"type": "number"}
    },
    "required": ["action"]
}

# 3.3 Schema 驅動 JSON 輸出格式 - 導航
response_schema_navigation = {
    "type": "object",
    "properties": {
        "action": {
            "type": "string",
            "enum": ["go_to_location"]
        },
        "value": {"type": "number"}
    },
    "required": ["action"]
}

# 3.4 建立 Gemini 模型物件 - 基本移動
model_basic = genai.GenerativeModel(
    "gemini-2.5-pro",
    generation_config={
        "response_mime_type": "application/json",
        "response_schema": response_schema_basic
    }
)

# 3.5 建立 Gemini 模型物件 - 導航
model_navigation = genai.GenerativeModel(
    "gemini-2.5-pro",
    generation_config={
        "response_mime_type": "application/json",
        "response_schema": response_schema_navigation
    }
)

# ====================================================
# ========== Step 4 Gemini API 初始化 =============
# ====================================================

# 4.1 初始化 Gemini 模型
def choose_microphone():
    """列出麥克風清單並讓使用者選擇"""
    print("🎤 可用的麥克風清單：")
    for i, mic_name in enumerate(sr.Microphone.list_microphone_names()):
        print(f"{i}: {mic_name}")
    mic_index = int(input("請輸入要使用的麥克風 index: "))
    print(f"✅ 已選擇麥克風 index {mic_index}")
    return mic_index

# 4.2 測試函式
def test_microphone(mic_index):
    r = sr.Recognizer()
    with sr.Microphone(device_index=13, sample_rate=16000) as source:  # 12 是 pulse

        print("🎙️ Step 1: 請講話...")
        r.adjust_for_ambient_noise(source)
        try:
            audio = r.listen(source, timeout=5)
            print("🎧 Step 2: 錄音完成，辨識中...")
            text = r.recognize_google(audio, language="zh-TW")
            print("🧠 辨識結果：", text)
        except sr.WaitTimeoutError:
            print("❌ 等待超時，沒偵測到語音。")
        except sr.UnknownValueError:
            print("🤷 無法辨識語音內容")
        except sr.RequestError as e:
            print("🌐 語音服務錯誤：", e)

# 4.3 測試語音辨識
def test_speech_recognition(mic_index):
    """測試 Google 語音辨識"""
    r = sr.Recognizer()
    with sr.Microphone(device_index=mic_index) as source:
        r.adjust_for_ambient_noise(source)
        print("🎙️ Step 2: 請講話...")
        audio = r.listen(source)
    text = r.recognize_google(audio, language="zh-TW")
    print("✅ 辨識結果:", text)
    return text

# 4.4 測試 Gemini JSON 輸出 - 基本移動
def test_gemini_json():
    """手動輸入文字，測試 Gemini JSON 輸出"""
    command = input("🎯 Step 3: 請輸入基本移動指令 (例如：前進 5 公尺): ")
    response = model_basic.generate_content(f"將以下中文指令轉換為機器人基本移動指令。支援的動作：move_forward(前進), move_backward(後退), turn_left(左轉), turn_right(右轉), stop(停止)。指令：{command}")
    cmd = json.loads(response.text)
    print("✅ Gemini 輸出 JSON:", cmd)
    return cmd

# 4.5 完整流程測試 - 基本移動
def run_integration(mic_index, ros_controller=None):
    r = sr.Recognizer()
    with sr.Microphone(device_index=mic_index) as source:
        r.adjust_for_ambient_noise(source)
        print("🎙️ Step 4: 請講出基本移動指令...")
        audio = r.listen(source)

    text = r.recognize_google(audio, language="zh-TW")
    print("📝 辨識文字:", text)

    try:
        response = model_basic.generate_content(f"將以下中文指令轉換為機器人基本移動指令。支援的動作：move_forward(前進), move_backward(後退), turn_left(左轉), turn_right(右轉), stop(停止)。指令：{text}")
        cmd = json.loads(response.text)
        print("📦 Gemini 強制 JSON 輸出:", cmd)
        execute_command_with_ros(cmd, ros_controller)   # <── 傳進去
    except Exception as e:
        print("⚠️ 解析失敗:", e)

# 4.6 語音導航循環
def run_voice_navigation(mic_index, ros_controller=None):
    """持續語音導航模式"""
    print("🎯 進入語音導航模式，說「停止」結束程式")
    r = sr.Recognizer()
    
    while True:
        try:
            with sr.Microphone(device_index=mic_index) as source:
                r.adjust_for_ambient_noise(source)
                print("🎙️ 請說出導航指令 (例如：去1號點位)...")
                audio = r.listen(source, timeout=10)

            text = r.recognize_google(audio, language="zh-TW")
            print("📝 辨識文字:", text)
            
            # 檢查是否要退出
            if "停止" in text or "結束" in text or "退出" in text:
                print("👋 結束語音導航")
                break

            try:
                response = model_navigation.generate_content(f"將以下中文指令轉換為機器人導航指令。支援的動作：go_to_location(導航到指定點位)。如果用戶說「去1號點位」或「前往點位2」等，使用go_to_location動作，value設為點位編號。指令：{text}")
                cmd = json.loads(response.text)
                print("📦 Gemini 輸出 JSON:", cmd)
                execute_command_with_ros(cmd, ros_controller)
            except Exception as e:
                print("⚠️ 解析失敗:", e)
                
        except sr.WaitTimeoutError:
            print("⏰ 等待超時，繼續監聽...")
        except sr.UnknownValueError:
            print("🤷 無法辨識語音內容")
        except sr.RequestError as e:
            print("🌐 語音服務錯誤：", e)
        except KeyboardInterrupt:
            print("👋 使用者中斷，結束程式")
            break

def get_default_mic_index():
    p = pyaudio.PyAudio()
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        if info["maxInputChannels"] > 0:
            print(f"使用麥克風裝置：Index {i} - {info['name']}")
            return i
    raise RuntimeError("找不到任何可用的麥克風裝置")
# ====================================================
# ========== Step 4 發送 ROS 指令 =============
# ====================================================

# 4.1 初始化 ROS2 與 RobotController 並執行指令
def execute_command_with_ros(cmd, ros_controller=None):
    action = cmd.get("action")
    value = cmd.get("value", 0)

    if CONTROL_ROBOT and ros_controller is not None:
        if action == "go_to_location":
            ros_controller.navigate_to_waypoint(int(value))
        else:
            ros_controller.send_command(action, value)
    else:
        if action == "move_forward":
            print(f"🚗 車子前進 {value} 公尺")
        elif action == "move_backward":
            print(f"🚗 車子後退 {value} 公尺")
        elif action == "turn_left":
            print(f"🚗 車子左轉 {value} 度")
        elif action == "turn_right":
            print(f"🚗 車子右轉 {value} 度")
        elif action == "stop":
            print("🚗 車子停止")
        elif action == "go_to_location":
            print(f"🎯 導航到點位 {int(value)}")
        else:
            print("❌ 未知指令:", action)


# ====================================================
# ========== Step 5 主程式 =============
# ====================================================
if __name__ == "__main__":
    #mic_index = choose_microphone()   # 互動選擇麥克風
    #mic_index = get_default_mic_index()   # ← 使用預設麥克風
    mic_index = 12   # ← 指定麥克風 index（nvidia 的 pulse audio）
    if CONTROL_ROBOT:
        rclpy.init()
        ros_controller = RobotController()
    else:
        ros_controller = None

    print("=== 測試選單 ===")
    print("1. 測試麥克風")
    print("2. 測試語音辨識")
    print("3. 測試 Gemini JSON 輸出 (基本移動)")
    print("4. 執行基本移動指令")
    print("5. 語音導航模式")
    choice = input("請輸入選項 (1-5): ")

    if choice == "1":
        test_microphone(mic_index)
    elif choice == "2":
        test_speech_recognition(mic_index)
    elif choice == "3":
        test_gemini_json()
    elif choice == "4":
        run_integration(mic_index, ros_controller)
    elif choice == "5":
        run_voice_navigation(mic_index, ros_controller)
    else:
        print("❌ 無效選項")
        
    if CONTROL_ROBOT:
        try:
            rclpy.spin(ros_controller)   # 保持 ROS2 node 活著
        except KeyboardInterrupt:
            pass
        finally:
            ros_controller.destroy_node()
            rclpy.shutdown()
