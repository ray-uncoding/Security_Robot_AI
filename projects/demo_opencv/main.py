import cv2

print("OpenCV 版本:", cv2.__version__)
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # 嘗試打開 USB 攝影機 /dev/video0
if not cap.isOpened():
    raise SystemExit("❌ 開不起 /dev/video0（USB 攝影機）")

while True:
    ok, frame = cap.read()
    if not ok: break
    cv2.imshow("USB Cam", frame)
    if cv2.waitKey(1) & 0xFF == 27:  # ESC 離開
        break

cap.release()
cv2.destroyAllWindows()
