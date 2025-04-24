from detector import PersonDetector
from reid_model import ReIDExtractor
from dynamic_db import DynamicFeatureDB

class ReIDManager:
    def __init__(self):
        self.detector = PersonDetector()
        self.reid = ReIDExtractor()
        self.db = DynamicFeatureDB()

    def process_frame(self, frame):
        """處理單幀影像，返回檢測與識別結果"""
        boxes, crops = self.detector.detect(frame)
        results = []

        for (box, crop) in zip(boxes, crops):
            feature = self.reid.extract(crop)
            if feature is None:
                continue

            person_id, score, is_new = self.db.match_or_register(feature)
            results.append((box, person_id, score, is_new))

        return results