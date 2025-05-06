import websocket
import threading
import time
import requests
import cv2
import numpy as np
import json

class AX8Manager:
    def __init__(self, url, username, password):
        self.url = url
        self.username = username
        self.password = password
        self.session = None
        self.running = False

    def login(self):
        """
        登錄到 AX8 相機
        """
        login_url = self.url.replace("/snapshot.jpg", "/check_login")
        self.session = requests.Session()
        payload = {
            "_username": self.username,
            "_password": self.password
        }
        try:
            response = self.session.post(login_url, data=payload, timeout=5)
            if response.status_code == 200:
                print("[AX8Manager] 登錄成功")
                return True
            else:
                print(f"[AX8Manager] 登錄失敗，HTTP 狀態碼: {response.status_code}")
        except Exception as e:
            print(f"[AX8Manager] 登錄時發生錯誤: {e}")
        return False

    def fetch_frame(self):
        """
        抓取單幀影像
        :return: OpenCV 格式的影像幀，或 None（如果抓取失敗）
        """
        if not self.session:
            print("[AX8Manager] 尚未登錄，無法抓取影像")
            return None

        try:
            response = self.session.get(self.url, stream=True, timeout=2)
            if response.status_code == 200:
                img_arr = np.asarray(bytearray(response.content), dtype=np.uint8)
                frame = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)
                return frame
            else:
                print(f"[AX8Manager] 無法抓取影像，HTTP 狀態碼: {response.status_code}")
        except Exception as e:
            print(f"[AX8Manager] 抓取影像時發生錯誤: {e}")
        return None

    def display_stream(self):
        """
        顯示 AX8 的影像串流
        """
        if not self.login():
            print("[AX8Manager] 登錄失敗，無法顯示影像串流")
            return

        self.running = True
        print("[AX8Manager] 開始顯示影像串流，按 'q' 鍵退出")
        last_heartbeat = time.time()

        while self.running:
            frame = self.fetch_frame()
            if frame is not None:
                cv2.imshow("AX8 Snapshot", frame)

            # 定期發送心跳請求
            if time.time() - last_heartbeat > 10:  # 每 10 秒發送一次心跳
                self.keep_alive()
                last_heartbeat = time.time()

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
            time.sleep(0.5)

        cv2.destroyAllWindows()
        print("[AX8Manager] 影像串流已停止")

    def keep_alive(self):
        """
        定期向 /camera/state 發送請求，保持會話活躍
        """
        keep_alive_url = self.url.replace("/snapshot.jpg", "/camera/state")
        try:
            response = self.session.get(keep_alive_url, timeout=5)
            if response.status_code == 200:
                print("[AX8Manager] 心跳請求成功，會話保持活躍")
            else:
                print(f"[AX8Manager] 心跳請求失敗，HTTP 狀態碼: {response.status_code}")
        except Exception as e:
            print(f"[AX8Manager] 心跳請求時發生錯誤: {e}")

    def stop(self):
        """
        停止影像串流
        """
        self.running = False