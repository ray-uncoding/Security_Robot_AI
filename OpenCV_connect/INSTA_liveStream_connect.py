import requests
import datetime
import json
import threading
import time
import cv2
import tkinter as tk
from tkinter import messagebox
from tkinter.scrolledtext import ScrolledText
from PIL import Image, ImageTk
import subprocess
import os

COMMAND_URL = "http://192.168.1.188:20000/osc/commands/execute"
STATE_URL = "http://192.168.1.188:20000/osc/state"
FILE_URL = "http://192.168.1.188:8000/fileuri"
PREVIEW_URL = "rtmp://192.168.1.188:1935/live/preview"

# UI 設定
root = tk.Tk()
root.title("Insta360 相機管理")
root.geometry("800x600")  # 設定較大的視窗尺寸
root.minsize(700, 500)  # 設定最小尺寸

# **相機狀態 UI**
status_label = tk.Label(root, text="請按下按鈕連接相機", font=("Arial", 16))
status_label.pack(pady=10)

response_text = ScrolledText(root, height=10, width=80, wrap=tk.WORD, font=("Arial", 12))
response_text.pack(pady=10, padx=20, expand=True, fill="both")

# **新增 直播訊息 UI**
live_status_label = tk.Label(root, text="直播狀態", font=("Arial", 14))
live_status_label.pack(pady=10)

live_response_text = ScrolledText(root, height=6, width=80, wrap=tk.WORD, font=("Arial", 12))
live_response_text.pack(pady=10, padx=20, expand=True, fill="both")

# **系統狀態紀錄**
async_task_list = []
polling_thread_active = False  
fingerprint = ""  
stop_event = threading.Event()  

# RTMP 伺服器路徑 (你的 `rtmp.py` 位於 `python-rtmp-server` 資料夾)
RTMP_SERVER_PATH = os.path.join(os.path.dirname(__file__), "../python-rtmp-server/rtmp.py")

def update_response_text(text): # 更新UI上的回訊
    response_text.delete("1.0", tk.END)
    response_text.insert(tk.END, json.dumps(text, indent=4, ensure_ascii=False) if isinstance(text, dict) else text)

# **更新直播狀態 UI**
def update_live_response_text(text):
    live_response_text.delete("1.0", tk.END)
    live_response_text.insert(tk.END, json.dumps(text, indent=4, ensure_ascii=False) if isinstance(text, dict) else text)

def poll_camera_state(): # 刷新狀態，保持連線
    global polling_thread_active
    while polling_thread_active: #如果非同步任務還沒結束就不要執行詢問，直接退掉
        try:
            payload = {}  # 刷新狀態只要用憑證訪問STATE_URL就好
            headers = {
                "Fingerprint": fingerprint,  
                "Content-Type": "application/json"                
            }
            response = requests.post(STATE_URL, json=payload, headers=headers, timeout=5)
            state_data = response.json()

            if not polling_thread_active: #如果非同步任務於詢問完狀態後還沒結束就不要繼續執行，防止閃退
                break
            
            root.after(0, lambda: update_response_text(state_data))

        except requests.exceptions.RequestException as e:
            if not polling_thread_active:
                break
            root.after(0, lambda: update_response_text(f"無法獲取狀態: {e}"))

        stop_event.wait(1)
        
def start_polling(): # 啟動新執行緒來自動刷新狀態
    global polling_thread_active
    if polling_thread_active:  # 避免重複啟動
        return
    polling_thread_active = True
    stop_event.clear()  # 確保執行緒可以正常運行
    threading.Thread(target=poll_camera_state, daemon=True).start()

def fetch_async_result(sequence_id): # 獲取非同步指令的結果
    try:
        payload = {
            "name": "camera._getResult",
            "parameters": {
                "list_ids": [sequence_id]
            }
        }
        headers = {
            "Content-Type": "application/json",
            "Fingerprint": fingerprint  # 加入 Fingerprint
        }
        response = requests.post(COMMAND_URL, json=payload, headers=headers, timeout=5)
        result_data = response.json()

        if result_data.get("state") == "exception":
            messagebox.showerror("錯誤", f"非同步任務失敗: {json.dumps(result_data, indent=4, ensure_ascii=False)}")
        else:
            messagebox.showinfo("非同步任務完成", f"結果: {json.dumps(result_data, indent=4, ensure_ascii=False)}")

    except requests.exceptions.RequestException as e:
        messagebox.showerror("錯誤", f"無法獲取非同步結果: {e}")

def connect_camera(): # 連接相機函式
    global fingerprint
    try:
        current_time = datetime.datetime.now(datetime.UTC).strftime("%m%d%H%M%Y.%S")
        payload = {
            "name": "camera._connect",
            "parameters": {
                "hw_time": current_time,
                "time_zone": "GMT+8"
            }
        }
        headers = {"Content-Type": "application/json"}

        response = requests.post(COMMAND_URL, json=payload, headers=headers, timeout=10)
        response_data = response.json()

        root.after(0, lambda: update_response_text(response_data))

        if response_data.get("state") == "done":
            fingerprint = response_data["results"].get("Fingerprint", "")
            print(f"📌 Fingerprint: {fingerprint}")  # Debug

            root.after(0, lambda: status_label.config(text="✅ 相機連接成功！", fg="green"))
            root.after(0, lambda: disconnect_button.config(state=tk.NORMAL))  # 啟用斷線按鈕
            root.after(0, lambda: messagebox.showinfo("成功", "相機已成功連接！"))
            start_polling()
        else:
            root.after(0, lambda: disconnect_button.config(state=tk.DISABLED))  # 確保仍然不可用
            root.after(0, lambda: status_label.config(text="⚠️ 連接失敗", fg="red"))
            root.after(0, lambda: messagebox.showwarning("錯誤", "相機連接失敗，請檢查 API 回應"))

    except requests.exceptions.RequestException as e:
        root.after(0, lambda: status_label.config(text="❌ 連接錯誤", fg="red"))
        root.after(0, lambda: update_response_text(f"Error: {e}"))
        root.after(0, lambda: messagebox.showerror("錯誤", f"連接相機時發生錯誤: {e}"))

def disconnect_camera():
    global polling_thread_active
    polling_thread_active = False
    stop_event.set()  # 讓執行緒結束
    stop_event.wait(1)  # 非阻塞等待執行緒完全停止

    root.after(0, lambda: update_response_text("相機已斷開連線"))
    root.after(0, lambda: status_label.config(text="❌ 相機已斷線", fg="red"))
    root.after(0, lambda: disconnect_button.config(state=tk.DISABLED))
    root.after(0, lambda: connect_button.config(state=tk.NORMAL))
    
def start_live_stream():
    global live_url

    # **先啟動 RTMP 伺服器**
    #start_rtmp_server()
    #time.sleep(3)  # **等待 RTMP 伺服器啟動**

    payload = {
        "name": "camera._startLive",
        "parameters": {
            "origin": {
                "mime": "video/mp4",
                "width": 1920,
                "height": 1080,
                "framerate": 30.0,
                "bitrate": 5000000,
                "logMode": 0,
                "liveUrl": "rtmp://0.0.0.0/live",  # ✅ **確保這裡是 RTMP 伺服器的 IP**
                "saveOrigin": False
            },
            "audio": {
                "mime": "audio/aac",
                "sampleFormat": "s16",
                "channelLayout": "stereo",
                "samplerate": 48000,
                "bitrate": 128000
            }
        },
        "autoConnect": {
            "enable": True,
            "interval": 3000,
            "count": -1
        }
    }

    json_payload = json.dumps(payload)
    headers = {
        "Content-Type": "application/json",
        "Fingerprint": fingerprint
    }

    response = requests.post(COMMAND_URL, json=payload, headers=headers, timeout=10)
    
    try:
        response_data = response.json()
        update_live_response_text(response_data)  # ✅ 顯示 API 回應在 UI

        if response_data.get("state") == "done":
            live_url = response_data["results"].get("_liveUrl", "")
            if live_url:
                messagebox.showinfo("成功", f"直播已啟動，串流網址: {live_url}")
                display_rtsp_stream(live_url)  # ✅ 顯示直播畫面
            else:
                messagebox.showerror("錯誤", "未獲取到直播串流網址")
        else:
            messagebox.showerror("錯誤", "啟動直播失敗")

    except json.JSONDecodeError:
        messagebox.showerror("錯誤", "無法解析 API 回應")
        update_live_response_text(response.text)



# **顯示 RTSP 串流**
def display_rtsp_stream(rtsp_url):
    cap = cv2.VideoCapture(rtsp_url)
    
    if not cap.isOpened():
        messagebox.showerror("錯誤", "無法開啟 RTSP 串流")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ 串流中斷")
            break

        cv2.imshow("Insta360 直播", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    
    
def start_rtmp_server():
    """
    啟動 RTMP 伺服器
    """
    try:
        print("🚀 啟動 RTMP 伺服器...")
        subprocess.Popen(["python", RTMP_SERVER_PATH], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print("✅ RTMP 伺服器已啟動")
    except Exception as e:
        print(f"❌ 無法啟動 RTMP 伺服器: {e}")


# 創建按鈕（置中）
connect_button = tk.Button(root, text="連接相機", font=("Arial", 14), command=connect_camera)
connect_button.pack(pady=15)

disconnect_button = tk.Button(root, text="斷開連線", font=("Arial", 14), command=disconnect_camera)
disconnect_button.pack(pady=15)
disconnect_button.config(state=tk.DISABLED)  # 預設為不可點擊

live_button = tk.Button(root, text="開啟直播", font=("Arial", 14), command=start_live_stream)
live_button.pack(pady=15)

# 啟動 UI
root.mainloop()
