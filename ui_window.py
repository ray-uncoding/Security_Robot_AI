from PyQt5.QtWidgets import (
    QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
    QLabel, QTextEdit, QGroupBox, QSizePolicy
)
from PyQt5.QtCore import QTimer
from shared_queue import (
    log_queue_camera, log_queue_stream, log_queue_reid, log_queue_system,
    stop_event
)
import threading
from core import start_all_threads

class ControlPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("保全機器人 人工智慧控制台")
        self.resize(1080, 720)
        self.threads = []

        self.start_btn = QPushButton("啟動系統")
        self.stop_btn = QPushButton("停止系統")
        self.status_label = QLabel("系統狀態：未啟動")
        self.status_label.setStyleSheet("font-weight: bold; color: #555")

        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self.start_btn)
        btn_layout.addWidget(self.stop_btn)
        btn_layout.addStretch()
        btn_layout.addWidget(self.status_label)

        self.log_camera = self.create_log_box("📷 相機訊息")
        self.log_stream = self.create_log_box("📡 串流訊息")
        self.log_reid = self.create_log_box("🔍 身份識別結果")
        self.log_system = self.create_log_box("🛠️ 系統訊息")

        log_layout_top = QHBoxLayout()
        log_layout_top.addWidget(self.log_camera["group"])
        log_layout_top.addWidget(self.log_stream["group"])

        log_layout_bottom = QHBoxLayout()
        log_layout_bottom.addWidget(self.log_reid["group"])
        log_layout_bottom.addWidget(self.log_system["group"])

        main_layout = QVBoxLayout()
        main_layout.addLayout(btn_layout)
        main_layout.addLayout(log_layout_top)
        main_layout.addLayout(log_layout_bottom)
        self.setLayout(main_layout)

        self.start_btn.clicked.connect(self.start_threads)
        self.stop_btn.clicked.connect(self.stop_threads)

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
        if not any(t.is_alive() for t in threading.enumerate() if t.name.startswith("worker")):
            stop_event.clear()
            self.status_label.setText("系統狀態：執行中")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            start_all_threads()
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