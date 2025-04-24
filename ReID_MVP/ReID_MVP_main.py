import cv2
from detector import PersonDetector
from reid_model import ReIDExtractor
from dynamic_db import DynamicFeatureDB
import time

# 初始化模組
detector = PersonDetector()
reid = ReIDExtractor()
db = DynamicFeatureDB()

# 攝影機
cap = cv2.VideoCapture(0)
font = cv2.FONT_HERSHEY_SIMPLEX

while True:
    ret, frame = cap.read()
    if not ret:
        break

    boxes, crops = detector.detect(frame)

    for (box, crop) in zip(boxes, crops):
        feature = reid.extract(crop)
        if feature is None:
            continue

        person_id, score, is_new = db.match_or_register(feature)

        # 畫框與顯示 ID
        x1, y1, x2, y2 = box
        label = f"{person_id} ({score:.2f})"
        color = (0, 255, 0) if not is_new else (0, 165, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, label, (x1, y1 - 10), font, 0.5, color, 1)

    cv2.imshow("ReID MVP", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
