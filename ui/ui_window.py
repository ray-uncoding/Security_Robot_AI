import threading

import cv2
from PyQt5.QtCore import Qt, QTimer  # Added Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (  # Added QComboBox; Added QSplitter for better layout management and QApplication
    QApplication, QComboBox, QGroupBox, QHBoxLayout, QLabel, QPushButton,
    QSizePolicy, QSplitter, QTextEdit, QVBoxLayout, QWidget)

# from core import start_all_threads # Keep this if start_threads uses it directly
from core.core import start_all_threads, stop_all_threads
from ai.gemini_client import GeminiClient
from core.shared_queue import (
    camera_frame_queue, gemini_prompt_queue, gemini_response_queue,
    log_queue_camera, log_queue_gemini, log_queue_reid, log_queue_stream,
    log_queue_system, id_result_queue, stop_event
)

class ControlPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("保全機器人 人工智慧控制台 V2 (with Gemini)") # Updated title
        self.resize(1280, 800) # Increased size
        # Removed self.threads = [] as core.py manages threads

        # --- Top Control Buttons ---
        self.start_btn = QPushButton("🚀 啟動系統")
        self.stop_btn = QPushButton("🛑 停止系統")
        self.status_label = QLabel("系統狀態：未啟動")
        self.status_label.setStyleSheet("font-weight: bold; color: #555; padding: 5px;")

        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self.start_btn)
        btn_layout.addWidget(self.stop_btn)
        btn_layout.addStretch()
        btn_layout.addWidget(self.status_label)

        # --- Main Content Area (Splitter) ---
        main_splitter = QSplitter(Qt.Horizontal) # Split left (logs/cam) and right (gemini)

        # --- Left Panel (Logs and Camera) ---
        left_panel_widget = QWidget()
        left_layout = QVBoxLayout(left_panel_widget)
        left_layout.setContentsMargins(0,0,0,0)

        # Camera View
        self.camera_view = QLabel("尚未啟動攝影機")
        self.camera_view.setMinimumSize(320, 240) # Use minimum size
        self.camera_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding) # Allow expansion
        self.camera_view.setAlignment(Qt.AlignCenter)
        self.camera_view.setStyleSheet("background-color: black; color: white; font-size: 14px; border: 1px solid #555;")

        # Log Boxes (Top Row)
        self.log_camera = self.create_log_box("📷 相機訊息")
        self.log_stream = self.create_log_box("📡 串流訊息")

        log_layout_top = QHBoxLayout()
        log_layout_top.addWidget(self.log_camera["group"])
        log_layout_top.addWidget(self.log_stream["group"])

        # Log Boxes (Bottom Row)
        self.log_reid = self.create_log_box("🔍 身份識別結果")
        self.log_system = self.create_log_box("🛠️ 系統訊息")
        self.log_gemini = self.create_log_box("✨ Gemini AI 訊息") # New Gemini Log Box

        log_layout_bottom = QHBoxLayout()
        log_layout_bottom.addWidget(self.log_reid["group"])
        log_layout_bottom.addWidget(self.log_system["group"])
        log_layout_bottom.addWidget(self.log_gemini["group"]) # Add Gemini log box here

        # Assemble Left Panel
        left_layout.addWidget(self.camera_view)
        left_layout.addLayout(log_layout_top)
        left_layout.addLayout(log_layout_bottom)
        main_splitter.addWidget(left_panel_widget)

        # --- Right Panel (Gemini Interaction) ---
        right_panel_widget = QWidget()
        right_layout = QVBoxLayout(right_panel_widget)
        right_layout.setContentsMargins(5,5,5,5)

        gemini_group = QGroupBox("🤖 Gemini AI 互動")
        gemini_group.setStyleSheet("QGroupBox { font-weight: bold; margin-top: 10px; } QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 3px 10px; }")
        gemini_layout = QVBoxLayout()

        # Model Selection
        model_layout = QHBoxLayout()
        model_layout.addWidget(QLabel("選擇模型:"))
        self.gemini_model_combo = QComboBox()
        self.gemini_model_combo.addItems(GeminiClient.list_available_models()) # Populate models
        model_layout.addWidget(self.gemini_model_combo)
        gemini_layout.addLayout(model_layout)

        # Prompt Input
        gemini_layout.addWidget(QLabel("輸入提示:"))
        self.gemini_prompt_input = QTextEdit()
        self.gemini_prompt_input.setPlaceholderText("在這裡輸入你想問 Gemini 的問題...")
        self.gemini_prompt_input.setFixedHeight(100) # Set fixed height for input
        gemini_layout.addWidget(self.gemini_prompt_input)

        # Send Button
        self.gemini_send_btn = QPushButton("✉️ 發送訊息")
        self.gemini_send_btn.clicked.connect(self.send_gemini_prompt)
        gemini_layout.addWidget(self.gemini_send_btn)

        # Response Output
        gemini_layout.addWidget(QLabel("Gemini 回應:"))
        self.gemini_response_output = QTextEdit()
        self.gemini_response_output.setReadOnly(True)
        self.gemini_response_output.setStyleSheet("background-color: #e9f5ff;") # Light blue background
        gemini_layout.addWidget(self.gemini_response_output) # Takes remaining space

        gemini_group.setLayout(gemini_layout)
        right_layout.addWidget(gemini_group)
        main_splitter.addWidget(right_panel_widget)

        # --- Final Layout Assembly ---
        main_layout = QVBoxLayout()
        main_layout.addLayout(btn_layout)
        main_layout.addWidget(main_splitter) # Add the splitter
        self.setLayout(main_layout)

        # Set initial splitter sizes (optional)
        main_splitter.setSizes([int(self.width() * 0.65), int(self.width() * 0.35)]) # Adjust ratio as needed

        # --- Connections & Timers ---
        self.start_btn.clicked.connect(self.start_threads_ui) # Renamed method
        self.stop_btn.clicked.connect(self.stop_threads_ui)   # Renamed method

        # Log Update Timers (Adjust intervals if needed)
        self.timer_log_update = QTimer()
        self.timer_log_update.timeout.connect(self.update_all_logs)
        self.timer_log_update.start(300) # Update all logs less frequently

        # Camera View Timer
        self.timer_camera_view = QTimer()
        self.timer_camera_view.timeout.connect(self.update_camera_view)
        self.timer_camera_view.start(33)  # ~30 fps

        # Gemini Response Timer
        self.timer_gemini_response = QTimer()
        self.timer_gemini_response.timeout.connect(self.update_gemini_response)
        self.timer_gemini_response.start(250) # Check for Gemini responses

        # Initial state
        self.stop_btn.setEnabled(False)


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
        log_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        return {"group": group, "widget": log_widget}

    def start_threads_ui(self):
        # Check using threading.enumerate() or a flag if core doesn't expose threads list
        if not any(t.name.startswith("worker_") and t.is_alive() for t in threading.enumerate()):
            log_queue_system.put("[UI] Requesting thread start...")
            start_all_threads() # Call the function from core
            self.status_label.setText("系統狀態：執行中 ✅")
            self.status_label.setStyleSheet("color: green; font-weight: bold; padding: 5px;")
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
            log_queue_system.put("[UI] Start request sent.") # Log from UI perspective
        else:
            log_queue_system.put("[UI] Threads seem to be already running.")


    def stop_threads_ui(self):
        log_queue_system.put("[UI] Requesting thread stop...")
        stop_all_threads() # Call the function from core
        self.status_label.setText("系統狀態：已停止 🛑")
        self.status_label.setStyleSheet("color: red; font-weight: bold; padding: 5px;")
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        # Optionally clear queues or reset UI elements here if needed upon stop
        self.camera_view.setText("攝影機已停止")
        self.camera_view.setStyleSheet("background-color: black; color: orange; font-size: 14px; border: 1px solid #555;")
        log_queue_system.put("[UI] Stop request sent.")

    # --- Consolidated Log Update ---
    def update_all_logs(self):
        """Updates all log boxes from their respective queues."""
        log_map = {
            log_queue_camera: self.log_camera["widget"],
            log_queue_stream: self.log_stream["widget"],
            log_queue_reid: self.log_reid["widget"],
            log_queue_system: self.log_system["widget"],
            log_queue_gemini: self.log_gemini["widget"], # Added Gemini log
        }
        for q, widget in log_map.items():
            while not q.empty():
                try:
                    msg = q.get_nowait() # Use non-blocking get
                    widget.append(str(msg)) # Ensure msg is string
                    widget.verticalScrollBar().setValue(widget.verticalScrollBar().maximum()) # Auto-scroll
                except queue.Empty:
                    break # Should not happen with outer check, but safe
                except Exception as e:
                    print(f"Error updating log widget {widget.parent().title()}: {e}") # Debug print

    # --- Gemini Specific Methods ---
    def send_gemini_prompt(self):
        """Sends the prompt from the input box to the Gemini worker queue."""
        prompt_text = self.gemini_prompt_input.toPlainText().strip()
        selected_model = self.gemini_model_combo.currentText()

        if not prompt_text:
            # Maybe show a small status message instead of logging
            self.gemini_response_output.setText("請先輸入提示訊息。")
            return

        if not selected_model:
             self.gemini_response_output.setText("請先選擇一個模型。")
             return

        # Clear previous response and show waiting message
        self.gemini_response_output.setText("⏳ 正在等待 Gemini 回應...")
        QApplication.processEvents() # Force UI update

        prompt_data = {"prompt": prompt_text, "model": selected_model}
        gemini_prompt_queue.put(prompt_data)
        log_queue_gemini.put(f"[UI] Sent prompt to worker (Model: {selected_model}).")
        # Optionally disable send button while waiting
        # self.gemini_send_btn.setEnabled(False)

    def update_gemini_response(self):
        """Checks for responses from the Gemini worker and updates the UI."""
        while not gemini_response_queue.empty():
            try:
                response_text = gemini_response_queue.get_nowait()
                self.gemini_response_output.setText(response_text)
                # Re-enable send button
                # self.gemini_send_btn.setEnabled(True)
            except queue.Empty:
                break
            except Exception as e:
                 self.gemini_response_output.setText(f"Error displaying response: {e}")
                 log_queue_gemini.put(f"[UI] Error updating Gemini response widget: {e}")


    # --- Camera View Update ---
    def update_camera_view(self):
        """更新攝影機畫面與檢測結果"""
        if not hasattr(self, "last_frame"):
            self.last_frame = None  # 初始化紀錄上次畫面

        latest_frame = None
        while not camera_frame_queue.empty():
            latest_frame = camera_frame_queue.get()

        # 若有新畫面，更新畫面與紀錄
        if latest_frame is not None:
            self.last_frame = latest_frame

        # 無論有無新畫面，都顯示 last_frame（除非一開始就沒有）
        if self.last_frame is not None:
            try:
                # 從 id_result_queue 獲取檢測結果
                while not id_result_queue.empty():
                    result = id_result_queue.get()
                    box = result["box"]
                    person_id = result["person_id"]
                    confidence = result["confidence"]

                    # 繪製檢測框與 ID 標籤
                    x1, y1, x2, y2 = map(int, box)
                    cv2.rectangle(self.last_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        self.last_frame,
                        f"{person_id} ({confidence:.2f})",
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1,
                        cv2.LINE_AA,
                    )

                # 將 OpenCV 的影像轉換為 QImage
                rgb_image = cv2.cvtColor(self.last_frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w
                qimg = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(qimg).scaled(
                    self.camera_view.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
                )
                self.camera_view.setPixmap(pixmap)
                self.camera_view.setAlignment(Qt.AlignCenter)
            except Exception as e:
                log_queue_system.put(f"[UI] Error updating camera view: {e}")
                self.camera_view.setText("🚫 畫面錯誤")
        elif not self.stop_btn.isEnabled():  # Only show "No Frame" if system is stopped
            self.camera_view.setText("🚫 無畫面")
            self.camera_view.setStyleSheet(
                "background-color: black; color: white; font-size: 14px; border: 1px solid #555;"
            )
            self.camera_view.setAlignment(Qt.AlignCenter)


    # Override closeEvent to ensure threads are stopped
    def closeEvent(self, event):
        """Ensure threads are stopped when the window is closed."""
        log_queue_system.put("[UI] Window close requested. Stopping threads...")
        self.stop_threads_ui()
        # Give threads a moment to stop (optional, daemon threads should exit anyway)
        # time.sleep(0.5)
        event.accept() # Accept the close event

    # Removed individual update_log_* methods as they are replaced by update_all_logs
    # def update_log_camera(self): ...
    # def update_log_stream(self): ...
    # def update_log_reid(self): ...
    # def update_log_system(self): ...
