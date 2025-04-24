from ultralytics import YOLO
import cv2
import torch

class PersonDetector:
    def __init__(self, model_path='yolov8n.pt', device=None):
        if device is None:
            device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = YOLO(model_path)
        self.model.to(device)
        self.device = device

    def detect(self, frame):
        results = self.model(frame, verbose=False)[0]
        person_boxes = []
        crops = []

        for box in results.boxes:
            cls = int(box.cls[0])
            if cls == 0:  # Class 0 = person in COCO dataset
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                person_boxes.append((x1, y1, x2, y2))
                crop = frame[y1:y2, x1:x2].copy()
                crops.append(crop)

        return person_boxes, crops


if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    detector = PersonDetector()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        boxes, crops = detector.detect(frame)

        for (x1, y1, x2, y2) in boxes:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.imshow("YOLO Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
