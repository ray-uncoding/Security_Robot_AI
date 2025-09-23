# ====================================================
# ========== Step 1 基本匯入與初始化 =============
# ====================================================

# 1.1 標準庫與第三方套件匯入
import speech_recognition as sr
import google.generativeai as genai
import json
import os
import time, math, threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# 1.2 全域變數，決定是否真的控制機器人
CONTROL_ROBOT = True   # True: 發送 /cmd_vel 控制車子；False: 只印出指令

# ====================================================
# ========== Step 2 機器人控制 =============
# ====================================================

# 2.1 機器人控制類別
class RobotController(Node):
    
    # 2.2 初始化
    def __init__(self):
        super().__init__('voice_command_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

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

# ====================================================
# ========== Step 3 Gemini API 初始化 =============
# ====================================================

# 3.1 初始化 Gemini 模型
genai.configure(api_key=os.environ.get("GEMINI_API_KEY", "YOUR_API_KEY"))

# 3.2 Schema 驅動 JSON 輸出格式
response_schema = {
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

# 3.3 建立 Gemini 模型物件
model = genai.GenerativeModel(
    "gemini-1.5-pro",
    generation_config={
        "response_mime_type": "application/json",
        "response_schema": response_schema
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
    """測試 PyAudio 能否正常錄音"""
    r = sr.Recognizer()
    with sr.Microphone(device_index=mic_index) as source:
        r.adjust_for_ambient_noise(source)
        print("🎙️ Step 1: 請講話...")
        audio = r.listen(source)
    print("✅ 麥克風 OK，錄到音訊長度:", len(audio.frame_data))

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

# 4.4 測試 Gemini JSON 輸出
def test_gemini_json():
    """手動輸入文字，測試 Gemini JSON 輸出"""
    command = input("🎯 Step 3: 請輸入指令 (例如：前進 5 公尺): ")
    response = model.generate_content(command)
    cmd = json.loads(response.text)
    print("✅ Gemini 輸出 JSON:", cmd)
    return cmd

# 4.5 完整流程測試
def run_integration(mic_index, ros_controller=None):
    r = sr.Recognizer()
    with sr.Microphone(device_index=mic_index) as source:
        r.adjust_for_ambient_noise(source)
        print("🎙️ Step 4: 請講出指令...")
        audio = r.listen(source)

    text = r.recognize_google(audio, language="zh-TW")
    print("📝 辨識文字:", text)

    try:
        response = model.generate_content(text)
        cmd = json.loads(response.text)
        print("📦 Gemini 強制 JSON 輸出:", cmd)
        execute_command_with_ros(cmd, ros_controller)   # <── 傳進去
    except Exception as e:
        print("⚠️ 解析失敗:", e)


# ====================================================
# ========== Step 4 發送 ROS 指令 =============
# ====================================================

# 4.1 初始化 ROS2 與 RobotController 並執行指令
def execute_command_with_ros(cmd, ros_controller=None):
    action = cmd.get("action")
    value = cmd.get("value", 0)

    if CONTROL_ROBOT and ros_controller is not None:
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
        else:
            print("❌ 未知指令:", action)


# ====================================================
# ========== Step 5 主程式 =============
# ====================================================
if __name__ == "__main__":
    #mic_index = choose_microphone()   # 互動選擇麥克風
    mic_index = 9   # ← 直接指定 index

    if CONTROL_ROBOT:
        rclpy.init()
        ros_controller = RobotController()
    else:
        ros_controller = None

    print("=== 測試選單 ===")
    print("1. 測試麥克風")
    print("2. 測試語音辨識")
    print("3. 測試 Gemini JSON 輸出")
    print("4. 執行整合版")
    choice = input("請輸入選項 (1-4): ")

    if choice == "1":
        test_microphone(mic_index)
    elif choice == "2":
        test_speech_recognition(mic_index)
    elif choice == "3":
        test_gemini_json()
    elif choice == "4":
        run_integration(mic_index, ros_controller)
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
