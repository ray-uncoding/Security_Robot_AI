import requests
import cv2

# Insta360 API 端點（用來啟動直播）
API_URL = "http://192.168.1.188:20000/osc/commands/execute"

# Insta360 內建 RTSP 串流地址（請確認相機的 RTSP 端口）
RTSP_URL = "rtsp://192.168.1.188:8554/live"

# 發送 API 請求來啟動直播
payload = {
    "name": "camera._startLive",
    "parameters": {
        "origin": {
            "mime": "video/mp4",
            "width": 3840,
            "height": 1920,
            "framerate": 30.0,
            "bitrate": 8000,
            "logMode": 0,
            "saveOrigin": False
        }
    }
}

headers = {"Content-Type": "application/json"}

try:
    response = requests.post(API_URL, json=payload, headers=headers, timeout=10)
    response_data = response.json()
    print("Response:", response_data)

    if response_data.get("state") == "done":
        print("直播已啟動，正在連接 RTSP 串流...")

        # 連接 Insta360 RTSP 串流
        cap = cv2.VideoCapture(RTSP_URL)
        if not cap.isOpened():
            print("無法打開 RTSP 串流，請檢查相機是否允許 RTSP")
        else:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("無法讀取影像，可能是串流未準備好")
                    break

                cv2.imshow("Insta360 Live Stream", frame)

                # 按 'q' 退出視窗
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            cap.release()
            cv2.destroyAllWindows()
    else:
        print("啟動直播失敗，請檢查 API 回應")
except requests.exceptions.RequestException as e:
    print("Error connecting to camera:", e)