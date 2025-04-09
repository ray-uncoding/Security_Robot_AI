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
root.geometry("800x600") 
root.minsize(700, 500)  

status_label = tk.Label(root, text="請按下按鈕連接相機", font=("Arial", 16))
status_label.pack(pady=10)

response_text = ScrolledText(root, height=10, width=80, wrap=tk.WORD, font=("Arial", 12))
response_text.pack(pady=10, padx=20, expand=True, fill="both")

live_status_label = tk.Label(root, text="直播狀態", font=("Arial", 14))
live_status_label.pack(pady=10)
live_response_text = ScrolledText(root, height=6, width=80, wrap=tk.WORD, font=("Arial", 12))
live_response_text.pack(pady=10, padx=20, expand=True, fill="both")

async_task_list = []
polling_thread_active = False  
fingerprint = ""  
stop_event = threading.Event()  

# RTMP 伺服器路徑 (你的 `rtmp.py` 位於 `python-rtmp-server` 資料夾)

RTMP_SERVER_PATH = os.path.join(os.path.dirname(__file__), "rtmp_server.py")

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

def connect_camera():
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
            root.after(0, lambda: disconnect_button.config(state=tk.NORMAL))
            root.after(0, lambda: messagebox.showinfo("成功", "相機已成功連接！"))
            start_polling()
        else:
            root.after(0, lambda: disconnect_button.config(state=tk.DISABLED))
            root.after(0, lambda: status_label.config(text="⚠️ 連接失敗", fg="red"))
            root.after(0, lambda: messagebox.showwarning("錯誤", "相機連接失敗，請檢查 API 回應"))

    except requests.exceptions.RequestException as e:
        root.after(0, lambda: status_label.config(text="❌ 連接錯誤", fg="red"))
        root.after(0, lambda: update_response_text(f"Error: {e}"))
        root.after(0, lambda: messagebox.showerror("錯誤", f"連接相機時發生錯誤: {e}"))

def disconnect_camera():
    global polling_thread_active
    polling_thread_active = False
    stop_event.set() 
    stop_event.wait(1)  

    root.after(0, lambda: update_response_text("相機已斷開連線"))
    root.after(0, lambda: status_label.config(text="❌ 相機已斷線", fg="red"))
    root.after(0, lambda: disconnect_button.config(state=tk.DISABLED))
    root.after(0, lambda: connect_button.config(state=tk.NORMAL))
    
def start_live_stream():

    payload_startPreview = {
        "name": "camera._startPreview",
        "parameters":{
            "origin":{
                "mime":"h264",
                "width":1920,
                "height":1440,
                "framerate":30,
                "bitrate":20480
            },
            "stiching":{
                "mode":"pano",
                "mime":"h264",
                "width":3840,
                "height":1920,
                "framerate":30, 
                "bitrate":10240
            },
        },
        "stabilization":True
    }
    
    headers = {
        "Content-Type": "application/json",
        "Fingerprint": fingerprint
    }

    response = requests.post(COMMAND_URL, json=payload_startPreview, headers=headers, timeout=10)
    
    response_data = response.json()
    update_live_response_text(response_data)
    
    
def OpenCAM():
    try:
        subprocess.run(["python", "IPcam_connect.py"])  # 啟動相機的子進程

    except json.JSONDecodeError:
        messagebox.showerror("錯誤", "無法解析 API 回應")

# 創建按鈕（置中）
connect_button = tk.Button(root, text="連接相機", font=("Arial", 14), command=connect_camera)
connect_button.pack(pady=15)

disconnect_button = tk.Button(root, text="斷開連線", font=("Arial", 14), command=disconnect_camera)
disconnect_button.pack(pady=15)
disconnect_button.config(state=tk.DISABLED)  # 預設為不可點擊

live_button = tk.Button(root, text="開啟直播", font=("Arial", 14), command=start_live_stream)
live_button.pack(pady=15)

live_button = tk.Button(root, text="開啟鏡頭", font=("Arial", 14), command=OpenCAM)
live_button.pack(pady=15)

# 啟動 UI
root.mainloop()
