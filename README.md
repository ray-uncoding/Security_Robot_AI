# Security_Robot_AI
 這是保全機器人的人機介面系統，以及智能系統

## 架構草稿

[RTMPWorker] → frame_queue
      ↓
[FaceDetectorWorker] → face_result_queue
      ↓
[ReIDWorker] → id_result_queue
      ↓
[UI 顯示] → log_queue_reid → QTextEdit
