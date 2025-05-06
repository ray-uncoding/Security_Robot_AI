from ultralytics import YOLO
import cv2
import numpy as np
import requests
import time

# 熱影像來源 URL
url = "http://admin:admin@192.168.1.100/snapshot.jpg"

# 載入模型（可用 yolov8n.pt 或專案提供的熱影像模型）
model = YOLO("yolov8n.pt")  # 或替換為熱影像專用模型


while True:
    try:
        resp = requests.get(url, stream=True, timeout=2)
        img_arr = np.asarray(bytearray(resp.content), dtype=np.uint8)
        frame = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)

        # 執行人偵測
        results = model.predict(frame, classes=[0], verbose=False)[0]  # class 0 = person

        # 畫出邊框
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, "person", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        cv2.imshow("AX8 + YOLO", frame)

    except Exception as e:
        print(f"Error: {e}")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    time.sleep(0.5)

cv2.destroyAllWindows()