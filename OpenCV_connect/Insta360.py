import requests
import cv2
import numpy as np

API_URL = "http://192.168.1.188:20000/osc/commands/execute"
RTSP_URL = "rtsp://192.168.1.188:8554/live"

payload = {
    "name": "camera._startLive",
    "parameters": {
        "stiching":{ 
            "mode": "",
            "mime": "",
            "width": 123,
            "height": 123,
            "framerate": 123,
            "bitrate": 123, 
            "map": "", 
            "_liveUrl": RTSP_URL,
            "liveOnHdmi": 0, 
            "fileSave": 0
        }
    },
    "audio":{
      "mime": "",
      "sampleFormat": "",
      "channelLayout": "",
      "samplerate": 123,
      "bitrate": 123
    },
    "autoConnect":{
        "enable": 1, 
        "interval": 123,
        "count": 123 
    }
}

headers = {"Content-Type": "application/json"}

try:
    response = requests.post(API_URL, json=payload, headers=headers, timeout=10)
    response_data = response.json()
    print("Response:", response_data)

    if response_data.get("state") == "done":
        print("直播已啟動，正在連接 RTSP 串流...")
        
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