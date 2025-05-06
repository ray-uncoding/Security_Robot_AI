import requests
import threading
import datetime

COMMAND_URL = "http://192.168.1.188:20000/osc/commands/execute"
STATE_URL = "http://192.168.1.188:20000/osc/state"

class InstaManager:
    def __init__(self):
        self.fingerprint = None
        self.polling_thread = None
        self.polling_active = False

    def connect_camera(self):
        """連接相機"""
        try:
            current_time = datetime.datetime.now(datetime.timezone.utc).strftime("%m%d%H%M%Y.%S")
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

            if response_data.get("state") == "done":
                self.fingerprint = response_data["results"].get("Fingerprint", "")
                print(f"[InstaManager] 相機連接成功，Fingerprint: {self.fingerprint}")
                return True
            else:
                print("[InstaManager] 相機連接失敗")
                return False
        except requests.exceptions.RequestException as e:
            print(f"[InstaManager] 連接相機時發生錯誤: {e}")
            return False

    def start_polling(self):
        """啟動狀態輪詢"""
        if self.polling_active:
            return
        self.polling_active = True
        self.polling_thread = threading.Thread(target=self._poll_camera_state, daemon=True)
        self.polling_thread.start()

    def _poll_camera_state(self):
        """輪詢相機狀態"""
        while self.polling_active:
            try:
                headers = {"Fingerprint": self.fingerprint, "Content-Type": "application/json"}
                response = requests.post(STATE_URL, headers=headers, timeout=5)
                state_data = response.json()
                print(f"[InstaManager] 相機狀態: {state_data}")
            except requests.exceptions.RequestException as e:
                print(f"[InstaManager] 獲取相機狀態時發生錯誤: {e}")
            threading.Event().wait(1)

    def stop_polling(self):
        """停止狀態輪詢"""
        self.polling_active = False
        if self.polling_thread:
            self.polling_thread.join()
            self.polling_thread = None

    def disconnect_camera(self):
        """斷開相機連線"""
        self.stop_polling()
        print("[InstaManager] 相機已斷開連線")