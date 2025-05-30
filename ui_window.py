"""UI window implementation for the Security Robot AI system.

This module provides the main GUI interface using PyQt5, including camera display,
log viewers, and Gemini AI interaction controls.
"""

import queue
import threading
from typing import Optional

import cv2
from PyQt5.QtCore import Qt, QTimer  # Added Qt
from PyQt5.QtGui import QImage, QPalette, QPixmap  # Added QPalette
from PyQt5.QtWidgets import (  # Added QComboBox; Added QSplitter for better layout management and QApplication
    QApplication, QButtonGroup, QComboBox, QGroupBox, QHBoxLayout, QLabel,
    QPushButton, QRadioButton, QSizePolicy, QSplitter, QTextEdit, QVBoxLayout,
    QWidget)

# from core import start_all_threads # Keep this if start_threads uses it directly
from core import start_all_threads  # Import stop function too
from core import stop_all_threads
from gemini_client import GeminiClient  # Import client for listing models
from shared_queue import (  # Added gemini log queue; Added gemini queues
    camera_frame_queue, gemini_prompt_queue, gemini_response_queue,
    log_queue_camera, log_queue_gemini, log_queue_reid, log_queue_stream,
    log_queue_system, stop_event)


class ControlPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("保全機器人 人工智慧控制台 V2 (with Gemini Live)") # Updated title
        self.resize(1280, 800) # Increased size
        # Removed self.threads = [] as core.py manages threads

        # --- Initialize GeminiClient for Live mode ---
        self.gemini_client = GeminiClient()
        
        # Live mode state tracking
        self.live_status = {
            'active': False,
            'video_mode': 'none',
            'muted': False
        }

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
        self.gemini_model_combo.addItems(self.gemini_client.list_available_models()) # Populate models
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

        # --- Live Mode Control Group ---
        live_group = QGroupBox("🎙️ Gemini Live 互動模式")
        live_group.setStyleSheet("QGroupBox { font-weight: bold; margin-top: 10px; } QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 3px 10px; }")
        live_layout = QVBoxLayout()

        # Live Mode Status
        self.live_status_label = QLabel("狀態：未啟動")
        self.live_status_label.setStyleSheet("font-weight: bold; color: #666; padding: 5px; background-color: #f0f0f0; border-radius: 3px;")
        live_layout.addWidget(self.live_status_label)

        # Video Mode Selection
        video_mode_layout = QHBoxLayout()
        video_mode_layout.addWidget(QLabel("視訊模式:"))
        
        self.video_mode_group = QButtonGroup()
        self.video_mode_none = QRadioButton("無視訊")
        self.video_mode_camera = QRadioButton("攝影機")
        self.video_mode_screen = QRadioButton("螢幕截圖")
        
        self.video_mode_none.setChecked(True)  # Default selection
        
        self.video_mode_group.addButton(self.video_mode_none, 0)
        self.video_mode_group.addButton(self.video_mode_camera, 1)
        self.video_mode_group.addButton(self.video_mode_screen, 2)
        
        video_mode_layout.addWidget(self.video_mode_none)
        video_mode_layout.addWidget(self.video_mode_camera)
        video_mode_layout.addWidget(self.video_mode_screen)
        video_mode_layout.addStretch()
        
        live_layout.addLayout(video_mode_layout)

        # Live Control Buttons
        live_control_layout = QHBoxLayout()
        
        self.live_start_btn = QPushButton("▶️ 啟動 Live 模式")
        self.live_start_btn.clicked.connect(self.start_live_mode)
        self.live_start_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; border-radius: 4px; }")
        
        self.live_stop_btn = QPushButton("⏹️ 停止 Live 模式")
        self.live_stop_btn.clicked.connect(self.stop_live_mode)
        self.live_stop_btn.setEnabled(False)
        self.live_stop_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; padding: 8px; border-radius: 4px; }")
        
        self.live_mute_btn = QPushButton("🔇 靜音")
        self.live_mute_btn.clicked.connect(self.toggle_mute)
        self.live_mute_btn.setEnabled(False)
        self.live_mute_btn.setStyleSheet("QPushButton { background-color: #FF9800; color: white; font-weight: bold; padding: 8px; border-radius: 4px; }")
        
        live_control_layout.addWidget(self.live_start_btn)
        live_control_layout.addWidget(self.live_stop_btn)
        live_control_layout.addWidget(self.live_mute_btn)
        
        live_layout.addLayout(live_control_layout)

        # Advanced Live Settings
        advanced_layout = QHBoxLayout()
        
        # Voice Selection
        advanced_layout.addWidget(QLabel("語音:"))
        self.voice_combo = QComboBox()
        self.voice_combo.addItems(["Zephyr", "Coral", "Nova", "Puck"])
        self.voice_combo.setCurrentText("Zephyr")
        advanced_layout.addWidget(self.voice_combo)
        
        # Response Modality
        advanced_layout.addWidget(QLabel("回應模式:"))
        self.response_mode_combo = QComboBox()
        self.response_mode_combo.addItems(["僅音訊", "音訊+文字"])
        advanced_layout.addWidget(self.response_mode_combo)
        
        advanced_layout.addStretch()
        live_layout.addLayout(advanced_layout)

        # Live Text Input (for sending text during live session)
        live_layout.addWidget(QLabel("Live 模式文字輸入:"))
        self.live_text_input = QTextEdit()
        self.live_text_input.setPlaceholderText("在 Live 模式中輸入文字訊息...")
        self.live_text_input.setFixedHeight(60)
        self.live_text_input.setEnabled(False)
        live_layout.addWidget(self.live_text_input)
        
        self.live_send_text_btn = QPushButton("📝 發送文字到 Live")
        self.live_send_text_btn.clicked.connect(self.send_text_to_live)
        self.live_send_text_btn.setEnabled(False)
        live_layout.addWidget(self.live_send_text_btn)

        live_group.setLayout(live_layout)
        right_layout.addWidget(live_group)
        
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

        # Live Mode Status Timer
        self.timer_live_status = QTimer()
        self.timer_live_status.timeout.connect(self.update_live_status)
        self.timer_live_status.start(500) # Check live mode status

        # Initial state
        self.stop_btn.setEnabled(False)


    def create_log_box(self, title):
        group = QGroupBox(title)
        log_widget = QTextEdit()
        log_widget.setReadOnly(True)

        # Get system palette for better theme compatibility
        palette = QApplication.palette()
        base_color = palette.color(QPalette.Base).name()
        text_color = palette.color(QPalette.Text).name()

        log_widget.setStyleSheet(f"""
            QTextEdit {{
                background-color: {base_color};
                color: {text_color};
                font-family: Consolas;
                font-size: 11pt;
                padding: 5px;
            }}
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
                
                # 檢查是否為配額錯誤
                if "配額錯誤:" in response_text:
                    # 使用特殊樣式顯示配額錯誤
                    self.gemini_response_output.setStyleSheet("background-color: #ffebee; color: #c62828;")
                    self.gemini_response_output.setText(response_text)
                    
                    # 在日誌中也顯示
                    log_queue_gemini.put("[UI] API 配額超限！請查看回應視窗中的詳細說明。")
                    
                elif "錯誤:" in response_text:
                    # 其他錯誤使用不同樣式
                    self.gemini_response_output.setStyleSheet("background-color: #fff3e0; color: #e65100;")
                    self.gemini_response_output.setText(response_text)
                    
                else:
                    # 正常回應恢復原本樣式
                    self.gemini_response_output.setStyleSheet("background-color: #e9f5ff;")
                    self.gemini_response_output.setText(response_text)
                    
                # Re-enable send button
                # self.gemini_send_btn.setEnabled(True)
            except queue.Empty:
                break
            except Exception as e:
                 self.gemini_response_output.setStyleSheet("background-color: #ffebee; color: #c62828;")
                 self.gemini_response_output.setText(f"顯示回應時發生錯誤: {e}")
                 log_queue_gemini.put(f"[UI] Error updating Gemini response widget: {e}")

    # --- Live Mode Methods ---
    def start_live_mode(self):
        """啟動 Gemini Live 模式"""
        if self.live_status['active']:
            log_queue_gemini.put("[UI] Live mode is already active.")
            return

        # 獲取選擇的視訊模式
        video_mode = "none"
        if self.video_mode_camera.isChecked():
            video_mode = "camera"
        elif self.video_mode_screen.isChecked():
            video_mode = "screen"

        # 獲取語音和回應模式設定
        voice_name = self.voice_combo.currentText()
        response_modalities = ["AUDIO"]
        if self.response_mode_combo.currentText() == "音訊+文字":
            response_modalities = ["AUDIO", "TEXT"]

        # 定義回調函數
        def on_text_received(text):
            log_queue_gemini.put(f"[Live] 收到文字: {text}")
            # 在 UI 中顯示收到的文字
            self.gemini_response_output.append(f"[Live回應] {text}")
            self.gemini_response_output.verticalScrollBar().setValue(
                self.gemini_response_output.verticalScrollBar().maximum()
            )

        def on_audio_received(audio_data):
            log_queue_gemini.put(f"[Live] 收到音訊資料: {len(audio_data)} bytes")

        try:
            # 顯示啟動中狀態
            self.live_status_label.setText("狀態：正在啟動 Live 模式...")
            self.live_status_label.setStyleSheet("font-weight: bold; color: #FF9800; padding: 5px; background-color: #fff3e0; border-radius: 3px;")
            self.live_start_btn.setEnabled(False)
            QApplication.processEvents()  # 立即更新 UI
            
            self.live_status['video_mode'] = video_mode
            success = self.gemini_client.start_live_session(
                video_mode=video_mode,
                voice_name=voice_name,
                response_modalities=response_modalities,
                on_text_received=on_text_received,
                on_audio_received=on_audio_received
            )
            
            if success:
                self.live_status['active'] = True
                self.update_live_ui_state()
                log_queue_gemini.put(f"[UI] Live 模式已啟動 (視訊: {video_mode}, 語音: {voice_name})")
            else:
                log_queue_gemini.put("[UI] Live 模式啟動失敗")
                self.live_start_btn.setEnabled(True)
                
                # 檢查是否為配額錯誤（從 log_queue_gemini 中獲取最新訊息）
                error_msg = "啟動失敗"
                try:
                    # 嘗試從日誌佇列獲取更詳細的錯誤訊息
                    temp_msgs = []
                    while not log_queue_gemini.empty():
                        msg = log_queue_gemini.get_nowait()
                        temp_msgs.append(msg)
                        if "配額錯誤" in str(msg) or "quota" in str(msg).lower():
                            error_msg = "配額超限 - 請檢查 API 使用量"
                            break
                        elif "not supported" in str(msg).lower() or "model not found" in str(msg).lower():
                            error_msg = "Live API 未啟用或不支援"
                            break
                    
                    # 將訊息放回佇列
                    for msg in temp_msgs:
                        log_queue_gemini.put(msg)
                        
                except:
                    pass
                
                self.live_status_label.setText(f"狀態：{error_msg}")
                self.live_status_label.setStyleSheet("font-weight: bold; color: #f44336; padding: 5px; background-color: #ffebee; border-radius: 3px;")
                
        except Exception as e:
            error_str = str(e)
            log_queue_gemini.put(f"[UI] Live 模式啟動錯誤: {error_str}")
            self.live_start_btn.setEnabled(True)
            
            # 分析錯誤類型
            if "quota" in error_str.lower() or "1011" in error_str:
                error_display = "配額超限"
                detailed_msg = "API 配額已用盡。請等待重置或升級方案。"
            elif "invalid api key" in error_str.lower():
                error_display = "API 金鑰無效"
                detailed_msg = "請檢查您的 API 金鑰設定。"
            elif "model not found" in error_str.lower():
                error_display = "模型不支援"
                detailed_msg = "Live API 可能未啟用或帳號無權限。"
            else:
                error_display = "啟動錯誤"
                detailed_msg = error_str[:100]
            
            self.live_status_label.setText(f"狀態：{error_display}")
            self.live_status_label.setStyleSheet("font-weight: bold; color: #f44336; padding: 5px; background-color: #ffebee; border-radius: 3px;")
            
            # 在回應區域顯示詳細錯誤
            self.gemini_response_output.setStyleSheet("background-color: #ffebee; color: #c62828;")
            self.gemini_response_output.setText(f"Live 模式啟動失敗\n\n錯誤類型：{error_display}\n詳細資訊：{detailed_msg}\n\n建議：使用 check_api_quota.py 工具診斷問題")

    def stop_live_mode(self):
        """停止 Gemini Live 模式"""
        if not self.live_status['active']:
            log_queue_gemini.put("[UI] No active live session to stop.")
            return

        try:
            success = self.gemini_client.stop_live_session()
            if success:
                self.live_status['active'] = False
                self.live_status['muted'] = False
                self.update_live_ui_state()
                log_queue_gemini.put("[UI] Live 模式已停止")
            else:
                log_queue_gemini.put("[UI] Live 模式停止失敗")
                
        except Exception as e:
            log_queue_gemini.put(f"[UI] Live 模式停止錯誤: {e}")

    def toggle_mute(self):
        """切換靜音狀態"""
        if not self.live_status['active']:
            return
        
        self.live_status['muted'] = not self.live_status['muted']
        
        if self.live_status['muted']:
            self.live_mute_btn.setText("🔊 取消靜音")
            log_queue_gemini.put("[UI] Live 模式已靜音")
        else:
            self.live_mute_btn.setText("🔇 靜音")
            log_queue_gemini.put("[UI] Live 模式已取消靜音")

    def send_text_to_live(self):
        """發送文字到 Live 模式"""
        if not self.live_status['active']:
            log_queue_gemini.put("[UI] Live 模式未啟動，無法發送文字")
            return

        text = self.live_text_input.toPlainText().strip()
        if not text:
            log_queue_gemini.put("[UI] 請輸入要發送的文字")
            return

        try:
            success = self.gemini_client.send_text_to_live(text)
            if success:
                self.live_text_input.clear()
                log_queue_gemini.put(f"[UI] 已發送文字到 Live 模式: {text[:50]}...")
            else:
                log_queue_gemini.put("[UI] 發送文字到 Live 模式失敗")
                
        except Exception as e:
            log_queue_gemini.put(f"[UI] 發送文字錯誤: {e}")

    def update_live_status(self):
        """更新 Live 模式狀態顯示"""
        try:
            # 檢查實際的 Live 模式狀態（這個方法不應該觸發連接）
            actual_active = self.gemini_client.is_live_mode_active()
            
            # 如果狀態不同步，更新本地狀態
            if actual_active != self.live_status['active']:
                log_queue_gemini.put(f"[UI] Live 狀態不同步，本地：{self.live_status['active']}，實際：{actual_active}")
                self.live_status['active'] = actual_active
                self.update_live_ui_state()
                
        except Exception as e:
            log_queue_gemini.put(f"[UI] 檢查 Live 狀態時發生錯誤: {e}")

    def update_live_ui_state(self):
        """更新 Live 模式 UI 狀態"""
        if self.live_status['active']:
            # Live 模式啟動狀態
            video_mode_text = {
                'none': '無視訊',
                'camera': '攝影機',
                'screen': '螢幕截圖'
            }.get(self.live_status['video_mode'], self.live_status['video_mode'])
            
            voice_name = self.voice_combo.currentText()
            self.live_status_label.setText(f"狀態：Live 模式運行中 ({video_mode_text}, {voice_name})")
            self.live_status_label.setStyleSheet("font-weight: bold; color: #4CAF50; padding: 5px; background-color: #e8f5e8; border-radius: 3px;")
            
            # 按鈕狀態
            self.live_start_btn.setEnabled(False)
            self.live_stop_btn.setEnabled(True)
            self.live_mute_btn.setEnabled(True)
            
            # 輸入控制項狀態
            self.live_text_input.setEnabled(True)
            self.live_send_text_btn.setEnabled(True)
            
            # 禁用設定選項
            self.video_mode_none.setEnabled(False)
            self.video_mode_camera.setEnabled(False)
            self.video_mode_screen.setEnabled(False)
            self.voice_combo.setEnabled(False)
            self.response_mode_combo.setEnabled(False)
            
            # 禁用傳統模式的輸入
            self.gemini_prompt_input.setEnabled(False)
            self.gemini_send_btn.setEnabled(False)
            self.gemini_model_combo.setEnabled(False)
            
        else:
            # Live 模式停止狀態
            self.live_status_label.setText("狀態：未啟動")
            self.live_status_label.setStyleSheet("font-weight: bold; color: #666; padding: 5px; background-color: #f0f0f0; border-radius: 3px;")
            
            # 按鈕狀態
            self.live_start_btn.setEnabled(True)
            self.live_stop_btn.setEnabled(False)
            self.live_mute_btn.setEnabled(False)
            self.live_mute_btn.setText("🔇 靜音")
            
            # 輸入控制項狀態
            self.live_text_input.setEnabled(False)
            self.live_send_text_btn.setEnabled(False)
            
            # 啟用設定選項
            self.video_mode_none.setEnabled(True)
            self.video_mode_camera.setEnabled(True)
            self.video_mode_screen.setEnabled(True)
            self.voice_combo.setEnabled(True)
            self.response_mode_combo.setEnabled(True)
            
            # 啟用傳統模式的輸入
            self.gemini_prompt_input.setEnabled(True)
            self.gemini_send_btn.setEnabled(True)
            self.gemini_model_combo.setEnabled(True)

    # --- Camera View Update ---
    def update_camera_view(self):
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
                rgb_image = cv2.cvtColor(self.last_frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w
                qimg = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
                # Scale pixmap keeping aspect ratio, fitting within the label's size
                pixmap = QPixmap.fromImage(qimg).scaled(
                    self.camera_view.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation) # Use KeepAspectRatio
                self.camera_view.setPixmap(pixmap)
                self.camera_view.setAlignment(Qt.AlignCenter) # Center the pixmap
            except Exception as e:
                log_queue_system.put(f"[UI] Error updating camera view: {e}")
                self.camera_view.setText("🚫 畫面錯誤")
        elif not self.stop_btn.isEnabled(): # Only show "No Frame" if system is stopped
             self.camera_view.setText("🚫 無畫面")
             self.camera_view.setStyleSheet("background-color: black; color: white; font-size: 14px; border: 1px solid #555;")
             self.camera_view.setAlignment(Qt.AlignCenter)


    # Override closeEvent to ensure threads are stopped
    def closeEvent(self, event):
        """Ensure threads are stopped when the window is closed."""
        log_queue_system.put("[UI] Window close requested. Stopping threads...")
        
        # 停止 Live 模式
        if self.live_status['active']:
            log_queue_system.put("[UI] Stopping Live mode before closing...")
            self.stop_live_mode()
        
        self.stop_threads_ui()
        # Give threads a moment to stop (optional, daemon threads should exit anyway)
        # time.sleep(0.5)
        event.accept() # Accept the close event

    # Removed individual update_log_* methods as they are replaced by update_all_logs
    # def update_log_camera(self): ...
    # def update_log_stream(self): ...
    # def update_log_reid(self): ...
    # def update_log_system(self): ...
