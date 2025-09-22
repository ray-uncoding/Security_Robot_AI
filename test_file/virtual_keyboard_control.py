# =================================================
# ========== Step 1 基本匯入與初始化 =============
# =================================================
import sys
import subprocess
import threading
import time
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QGridLayout, QLabel, QMessageBox
from PyQt5.QtCore import Qt
import pyautogui
from functools import partial
import speech_recognition as sr
import google.generativeai as genai
from pydantic import BaseModel
import os
import json

test_mode_without_ros2 = True  # 如果沒有 ROS2 環境，設為 True 可跳過 ROS2 啟動

# =================================================
# ========== Step 2 ROS2 指令與鍵盤對應 ==========
# =================================================

# 2.1 啟動 ROS2 底盤與鍵盤控制的指令
LAUNCH_CMD = ["ros2", "launch", "turn_on_wheeltec_robot", "turn_on_wheeltec_robot.launch.py"]
KEYBOARD_CMD = ["ros2", "run", "wheeltec_robot_keyboard", "wheeltec_keyboard"]

# 備註: 這兩個指令會在不同的終端視窗啟動

# 2.2 虛擬鍵盤對應按鍵（以 JSON 結構描述，方便擴充與訊號中繼）

KEY_MAP = {
    '前進': {"action": "move", "key": "w", "desc": "前進"},
    '後退': {"action": "move", "key": "s", "desc": "後退"},
    '左轉': {"action": "move", "key": "a", "desc": "左轉"},
    '右轉': {"action": "move", "key": "d", "desc": "右轉"},
    '停止': {"action": "stop", "key": "space", "desc": "停止"},
}


# =================================================
# ============ Step 3 Gemini 初始化 ============
# =================================================

# 3.1 Gemini 設定 API Key
genai.configure(api_key=os.environ.get("GEMINI_API_KEY", "YOUR_API_KEY_HERE"))
gemini_model = genai.GenerativeModel('gemini-1.5-pro')

# 備註: 
# 這個函式被呼叫後將會執行以下步驟
# 1. 根據是否選擇 ROI，決定使用全畫面或 ROI 區域進行 OCR
# 2. 使用 ImageProcessingService 進行品質增強
# 3. 呼叫 Gemini OCR API 進行辨識: 
# Gemini API Key 請設定在系統環境變數中，或直接替換上方 YOUR_API_KEY_HERE
# 可於 cmd 中執行: setx GEMINI_API_KEY "你的API金鑰"
# 或在 Linux/MacOS 終端機中執行: export GEMINI_API_KEY="你的API金鑰"
# 詳細請參考官方文件: https://developers.generativeai.google/products/gemini/get-started


# 3.2 定義 Gemini 指令格式 schema
class ChassisCommand(BaseModel):
    action: str
    key: str
    desc: str

# =================================================
# ========== Step 4 虛擬鍵盤 GUI 類別 ===========
# =================================================
class VirtualKeyboard(QWidget):

    # 4.1 初始化 GUI
    def __init__(self):
        super().__init__()
        self.setWindowTitle('ROS2 虛擬底盤鍵盤控制')
        self.setGeometry(200, 200, 350, 200)
        layout = QGridLayout()
        self.setLayout(layout)

        # 4.2 標籤
        label = QLabel('請先點選鍵盤控制終端視窗，再點擊下方按鈕')
        layout.addWidget(label, 0, 0, 1, 3)

        # 4.3 按鈕
        self.btn_up = QPushButton('↑ 前進')
        self.btn_down = QPushButton('↓ 後退')
        self.btn_left = QPushButton('← 左轉')
        self.btn_right = QPushButton('→ 右轉')
        self.btn_stop = QPushButton('■ 停止')        
        self.btn_voice = QPushButton('🎤 語音指令')
        
        layout.addWidget(self.btn_up, 1, 1)
        layout.addWidget(self.btn_left, 2, 0)
        layout.addWidget(self.btn_stop, 2, 1)
        layout.addWidget(self.btn_right, 2, 2)
        layout.addWidget(self.btn_down, 3, 1)
        layout.addWidget(self.btn_voice, 4, 1)
        
        # 4.4 綁定按鈕事件（用 partial 確保 self 正確）
        self.btn_up.clicked.connect(partial(self.send_key_json, '前進'))
        self.btn_down.clicked.connect(partial(self.send_key_json, '後退'))
        self.btn_left.clicked.connect(partial(self.send_key_json, '左轉'))
        self.btn_right.clicked.connect(partial(self.send_key_json, '右轉'))
        self.btn_stop.clicked.connect(partial(self.send_key_json, '停止'))
        self.btn_voice.clicked.connect(self.voice_command)


    # 4.5 處理按鈕事件，產生 JSON 並發送對應鍵盤訊號
    def send_key_json(self, action):
        
        cmd_json = json.dumps(KEY_MAP[action], ensure_ascii=False)  # 產生 JSON 字串
        print(f"[VIRTUAL_KEYBOARD] Send: {cmd_json}")
        cmd = KEY_MAP[action]                                       # 解析 JSON 並送出對應按鍵
        pyautogui.press(cmd["key"])                                 # pyautogui 會將按鍵訊號送到目前聚焦的視窗


    # 4.6 語音指令處理
    def voice_command(self):
        recognizer = sr.Recognizer()
        with sr.Microphone() as source:
            # 彈出提示
            QMessageBox.information(self, "語音輸入", "請開始說話...")
            try:
                audio = recognizer.listen(source, timeout=5, phrase_time_limit=5)
                text = recognizer.recognize_google(audio, language='zh-TW')
                print(f"[VOICE] 辨識結果: {text}")
            except Exception as e:
                QMessageBox.warning(self, "語音錯誤", f"語音辨識失敗: {e}")
                return

        # 呼叫 Gemini，強制回傳 JSON 格式
        prompt = f"請根據下列語音內容，直接回傳底盤控制 JSON 格式（action, key, desc），不要多餘說明。語音內容：{text}"
        try:
            resp = gemini_model.generate_content(
                [prompt],
                generation_config=genai.GenerationConfig(
                    response_mime_type="application/json",
                    response_schema=ChassisCommand
                )
            )
            # 解析 Gemini 回傳 JSON（自動驗證）
            if resp.candidates and resp.candidates[0].content and resp.candidates[0].content.parts:
                part = resp.candidates[0].content.parts[0]
                if part.function_call:
                    cmd = dict(part.function_call.args)
                else:
                    cmd = resp.candidates[0].content.parts[0].text
                    cmd = json.loads(cmd)
                print(f"[GEMINI] 回傳: {cmd}")
                pyautogui.press(cmd["key"])
                QMessageBox.information(self, "語音控制", f"已執行：{cmd['desc']}")
            else:
                QMessageBox.warning(self, "Gemini 回應錯誤", "未收到有效 JSON 指令。")
        except Exception as e:
            QMessageBox.warning(self, "Gemini 錯誤", f"Gemini API 失敗: {e}")


# =================================================
# ========== Step 5 ROS2 子程序啟動函式 ==========
# =================================================
def run_ros2_launch():
    subprocess.Popen(LAUNCH_CMD)

def run_ros2_keyboard():
    subprocess.Popen(KEYBOARD_CMD)


# =================================================
# ========== Step 6 主程式入口點 ================
# =================================================
def main():

    # 6.0 測試模式開關
    if not test_mode_without_ros2:
        # 6.1 如果有 ROS2 環境，啟動 ROS2 底盤與鍵盤控制（各自開一個 thread）
        # 如果沒有 ROS2 環境，設為 True 可跳過 ROS2 啟動
        threading.Thread(target=run_ros2_launch, daemon=True).start()
        time.sleep(2)  # 等待底盤啟動
        threading.Thread(target=run_ros2_keyboard, daemon=True).start()
        time.sleep(2)  # 等待鍵盤控制啟動

    # 6.2 啟動虛擬鍵盤 GUI
    app = QApplication(sys.argv)
    window = VirtualKeyboard()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
