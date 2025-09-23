import speech_recognition as sr
import google.generativeai as genai
import json
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# 全域變數，決定是否真的控制機器人
CONTROL_ROBOT = True

class RobotController(Node):
    def __init__(self):
        super().__init__('voice_command_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    def send_command(self, action, value=0.0):
        twist = Twist()

        if action == "move_forward":
            twist.linear.x = 0.2   # 固定前進速度 (m/s)
        elif action == "move_backward":
            twist.linear.x = -0.2
        elif action == "turn_left":
            twist.angular.z = 0.5  # 左轉 (rad/s)
        elif action == "turn_right":
            twist.angular.z = -0.5
        elif action == "stop":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            print("❌ 未知指令:", action)
            return

        self.publisher_.publish(twist)
        self.get_logger().info(f"✅ 已送出 ROS2 指令: {action}, value={value}")



# ==============================
# 初始化 Gemini 模型
# ==============================
genai.configure(api_key=os.environ.get("GEMINI_API_KEY", "YOUR_API_KEY"))

# Schema 驅動 JSON 輸出格式
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

model = genai.GenerativeModel(
    "gemini-1.5-pro",
    generation_config={
        "response_mime_type": "application/json",
        "response_schema": response_schema
    }
)

# ==============================
# Input 模組（麥克風/語音辨識）
# ==============================
def choose_microphone():
    """列出麥克風清單並讓使用者選擇"""
    print("🎤 可用的麥克風清單：")
    for i, mic_name in enumerate(sr.Microphone.list_microphone_names()):
        print(f"{i}: {mic_name}")
    mic_index = int(input("請輸入要使用的麥克風 index: "))
    print(f"✅ 已選擇麥克風 index {mic_index}")
    return mic_index

def test_microphone(mic_index):
    """測試 PyAudio 能否正常錄音"""
    r = sr.Recognizer()
    with sr.Microphone(device_index=mic_index) as source:
        r.adjust_for_ambient_noise(source)
        print("🎙️ Step 1: 請講話...")
        audio = r.listen(source)
    print("✅ 麥克風 OK，錄到音訊長度:", len(audio.frame_data))

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

# ==============================
# Gemini 模組（文字轉 JSON）
# ==============================
def test_gemini_json():
    """手動輸入文字，測試 Gemini JSON 輸出"""
    command = input("🎯 Step 3: 請輸入指令 (例如：前進 5 公尺): ")
    response = model.generate_content(command)
    cmd = json.loads(response.text)
    print("✅ Gemini 輸出 JSON:", cmd)
    return cmd

def run_integration(mic_index):
    """完整流程：語音 → 辨識文字 → Gemini JSON → 執行控制"""
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
        execute_command(cmd)
    except Exception as e:
        print("⚠️ 解析失敗，原始輸出:", response.text if 'response' in locals() else None)
        print("錯誤訊息:", e)

# ==============================
# 控制模組（模擬車子動作）
# ==============================
def execute_command(cmd):
    """依照 JSON 指令執行車子控制"""
    action = cmd.get("action")
    value = cmd.get("value", 0)

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

def execute_command_with_ros(cmd, ros_controller=None):
    action = cmd.get("action")
    value = cmd.get("value", 0)

    if CONTROL_ROBOT and ros_controller is not None:
        ros_controller.send_command(action, value)
    else:
        # 本地模擬輸出
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


# ==============================
# 主程式
# ==============================
if __name__ == "__main__":
    #mic_index = choose_microphone()   # ← 互動選擇麥克風
    mic_index = 4   # ← 直接指定 index

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
        run_integration(mic_index)
    else:
        print("❌ 無效選項")
