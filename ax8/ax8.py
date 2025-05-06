import cv2
import numpy as np
import requests
import time

url = "http://admin:admin@192.168.1.100/snapshot.jpg"

while True:
    try:
        response = requests.get(url, stream=True, timeout=2)
        if response.status_code == 200:
            img_arr = np.asarray(bytearray(response.content), dtype=np.uint8)
            frame = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)
            cv2.imshow("AX8 Snapshot", frame)
    except Exception as e:
        print(f"Error: {e}")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    time.sleep(0.5)  # 控制更新頻率

cv2.destroyAllWindows()
