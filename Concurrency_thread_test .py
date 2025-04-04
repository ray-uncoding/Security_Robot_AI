from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
    QLabel, QTextEdit, QGroupBox, QSizePolicy
)
from PyQt5.QtCore import QTimer
import sys
import threading
import time
import queue
import random

# 全域佇列與旗標
stop_event = threading.Event()
frame_queue = queue.Queue()
face_result_queue = queue.Queue()
id_result_queue = queue.Queue()

log_queue_camera = queue.Queue()
log_queue_stream = queue.Queue()
log_queue_system = queue.Queue()
log_queue_reid = queue.Queue()

# ---------- 模組定義（模擬資料流） ---------- #
def rtmp_worker():
    i = 0
    while not stop_event.is_set():
        frame = f"Frame_{i}"
        frame_queue.put(frame)
        log_queue_stream.put(f"[RTMPWorker] 推送 {frame}")
        time.sleep(1)
        i += 1

def face_detector_worker():
    while not stop_event.is_set():
        try:
            frame = frame_queue.get(timeout=1)
            face_features = {
                "frame": frame,
                "face_id": f"face_{random.randint(100,999)}",
                "embedding": [random.random() for _ in range(128)]
            }
            face_result_queue.put(face_features)
            log_queue_camera.put(f"[FaceDetector] 偵測出 {face_features['face_id']} from {frame}")
        except queue.Empty:
            continue

def reid_worker():
    while not stop_event.is_set():
        try:
            face_data = face_result_queue.get(timeout=1)
            person_id = f"Person_{random.randint(1, 5)}"
            result = {
                "frame": face_data["frame"],
                "face_id": face_data["face_id"],
                "person_id": person_id,
                "confidence": round(random.uniform(0.85, 0.99), 2)
            }
            id_result_queue.put(result)
            log_queue_reid.put(
                f"[ReID] {result['face_id']} ➜ {result['person_id']} (可信度: {result['confidence']})"
            )
        except queue.Empty:
            continue

# ---------- UI 主視窗 ---------- #
class ControlPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("保全機器人 人工智慧控制台")
        self.resize(1080, 720)
        self.threads = []

        # 控制按鈕與狀態列
        self.start_btn = QPushButton("啟動系統")
        self.stop_btn = QPushButton("停止系統")
        self.status_label = QLabel("系統狀態：未啟動")
        self.status_label.setStyleSheet("font-weight: bold; color: #555")

        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self.start_btn)
        btn_layout.addWidget(self.stop_btn)
        btn_layout.addStretch()
        btn_layout.addWidget(self.status_label)

        # 每個 log 區使用 GroupBox 包裝（統一配色）
        self.log_camera = self.create_log_box("📷 相機訊息")
        self.log_stream = self.create_log_box("📡 串流訊息")
        self.log_reid = self.create_log_box("🔍 身份識別結果")
        self.log_system = self.create_log_box("🛠️ 系統訊息")

        # log 分佈 layout
        log_layout_top = QHBoxLayout()
        log_layout_top.addWidget(self.log_camera["group"])
        log_layout_top.addWidget(self.log_stream["group"])

        log_layout_bottom = QHBoxLayout()
        log_layout_bottom.addWidget(self.log_reid["group"])
        log_layout_bottom.addWidget(self.log_system["group"])

        # 主 layout 結構
        main_layout = QVBoxLayout()
        main_layout.addLayout(btn_layout)
        main_layout.addLayout(log_layout_top)
        main_layout.addLayout(log_layout_bottom)
        self.setLayout(main_layout)

        # 事件綁定
        self.start_btn.clicked.connect(self.start_threads)
        self.stop_btn.clicked.connect(self.stop_threads)

        # UI 更新定時器
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_log_boxes)
        self.timer.start(500)

    def create_log_box(self, title):
        group = QGroupBox(title)
        log_widget = QTextEdit()
        log_widget.setReadOnly(True)
        log_widget.setStyleSheet("""
            QTextEdit {
                background-color: #f7f7f7;
                font-family: Consolas;
                font-size: 11pt;
                padding: 5px;
            }
        """)
        group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                margin-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 3px 10px;
            }
        """)
        layout = QVBoxLayout()
        layout.addWidget(log_widget)
        group.setLayout(layout)
        group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        return {"group": group, "widget": log_widget}

    def start_threads(self):
        if not any(t.is_alive() for t in self.threads):
            stop_event.clear()
            self.status_label.setText("系統狀態：執行中")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            self.threads = [
                threading.Thread(target=rtmp_worker, daemon=True),
                threading.Thread(target=face_detector_worker, daemon=True),
                threading.Thread(target=reid_worker, daemon=True),
            ]
            for t in self.threads:
                t.start()
            log_queue_system.put("[Main] 所有模組已啟動")

    def stop_threads(self):
        stop_event.set()
        self.status_label.setText("系統狀態：已停止")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        log_queue_system.put("[Main] 停止所有模組中...")

    def update_log_boxes(self):
        for queue_obj, log_widget in [
            (log_queue_camera, self.log_camera["widget"]),
            (log_queue_stream, self.log_stream["widget"]),
            (log_queue_reid, self.log_reid["widget"]),
            (log_queue_system, self.log_system["widget"]),
        ]:
            while not queue_obj.empty():
                msg = queue_obj.get()
                log_widget.append(msg)

# ---------- 執行應用 ---------- #
def run_app():
    app = QApplication(sys.argv)
    window = ControlPanel()
    window.show()
    sys.exit(app.exec_())

run_app()
