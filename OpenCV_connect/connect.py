import requests
import datetime
import json
import threading
import time
import tkinter as tk
from tkinter import messagebox
from tkinter.scrolledtext import ScrolledText

# Insta360 API 端點
API_URL = "http://192.168.1.188:20000/osc/commands/execute"
STATE_URL = "http://192.168.1.188:20000/osc/state"

# 創建主視窗
root = tk.Tk()
root.title("Insta360 相機管理")
root.geometry("700x500")  # 設定較大的視窗尺寸
root.minsize(600, 400)  # 設定最小尺寸

# 訊息顯示區域
status_label = tk.Label(root, text="請按下按鈕連接相機", font=("Arial", 16))
status_label.pack(pady=10)

# JSON 回應框（可滾動）
response_text = ScrolledText(root, height=10, width=80, wrap=tk.WORD, font=("Arial", 12))
response_text.pack(pady=10, padx=20, expand=True, fill="both")  # 自適應大小

# 記錄非同步任務的 sequence
async_task_list = []

# 自動刷新相機狀態的函式
def poll_camera_state():
    while polling_thread_active:
        try:
            response = requests.post(STATE_URL, json={}, timeout=5)
            state_data = response.json()

            # 更新 UI 顯示狀態
            response_text.delete("1.0", tk.END)
            response_text.insert(tk.END, json.dumps(state_data, indent=4, ensure_ascii=False))

            # 檢查是否有非同步任務完成
            if "_idRes" in state_data:
                completed_tasks = state_data["_idRes"]
                for task in completed_tasks:
                    if task in async_task_list:
                        async_task_list.remove(task)
                        fetch_async_result(task)

        except requests.exceptions.RequestException as e:
            response_text.delete("1.0", tk.END)
            response_text.insert(tk.END, f"無法獲取狀態: {e}")

        time.sleep(1)  # 每秒更新一次

# 啟動新執行緒來自動刷新狀態
def start_polling():
    global polling_thread_active
    polling_thread_active = True
    threading.Thread(target=poll_camera_state, daemon=True).start()

# 獲取非同步指令的結果
def fetch_async_result(sequence_id):
    try:
        payload = {
            "name": "camera._getResult",
            "parameters": {
                "list_ids": [sequence_id]
            }
        }
        response = requests.post(API_URL, json=payload, timeout=5)
        result_data = response.json()
        
        # 顯示結果
        messagebox.showinfo("非同步任務完成", f"結果: {json.dumps(result_data, indent=4, ensure_ascii=False)}")

    except requests.exceptions.RequestException as e:
        messagebox.showerror("錯誤", f"無法獲取非同步結果: {e}")

# 連接相機函式
def connect_camera():
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

        response = requests.post(API_URL, json=payload, headers=headers, timeout=10)
        response_data = response.json()

        # 顯示 JSON 回應
        response_text.delete("1.0", tk.END)
        response_text.insert(tk.END, json.dumps(response_data, indent=4, ensure_ascii=False))

        # 更新 UI 狀態
        if response_data.get("state") == "done":
            status_label.config(text="✅ 相機連接成功！", fg="green")
            disconnect_button.config(state=tk.NORMAL)  # 連接成功後啟用斷線按鈕
            messagebox.showinfo("成功", "相機已成功連接！")
            start_polling()  # 連接成功後，開始輪詢相機狀態
        else:
            status_label.config(text="⚠️ 連接失敗", fg="red")
            messagebox.showwarning("錯誤", "相機連接失敗，請檢查 API 回應")

    except requests.exceptions.RequestException as e:
        status_label.config(text="❌ 連接錯誤", fg="red")
        response_text.delete("1.0", tk.END)
        response_text.insert(tk.END, f"Error: {e}")
        messagebox.showerror("錯誤", f"連接相機時發生錯誤: {e}")

def disconnect_camera():
    global polling_thread_active
    polling_thread_active = False  # 停止輪詢執行緒

    try:
        payload = {"name": "camera._disconnect"}
        response = requests.post(API_URL, json=payload, timeout=5)
        response_data = response.json()

        response_text.delete("1.0", tk.END)
        response_text.insert(tk.END, json.dumps(response_data, indent=4, ensure_ascii=False))

        status_label.config(text="❌ 相機已斷線", fg="red")
        disconnect_button.config(state=tk.DISABLED)  # 斷線後禁用按鈕
        connect_button.config(state=tk.NORMAL)  # 啟用連接按鈕

    except requests.exceptions.RequestException as e:
        messagebox.showerror("錯誤", f"無法斷開相機: {e}")


# 創建按鈕（置中）
connect_button = tk.Button(root, text="連接相機", font=("Arial", 14), command=connect_camera)
connect_button.pack(pady=15)

disconnect_button = tk.Button(root, text="斷開連線", font=("Arial", 14), command=disconnect_camera)
disconnect_button.pack(pady=15)
disconnect_button.config(state=tk.DISABLED)  # 預設為不可點擊


# 啟動 UI
root.mainloop()
