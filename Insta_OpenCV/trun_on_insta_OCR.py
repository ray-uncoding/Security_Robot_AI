# ====================================================
# ========== Step 1 基本匯入與初始化 =============
# ====================================================

# 1.1 標準庫與第三方套件匯入
import sys
import os
import time
import cv2
import json
import logging
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QTextEdit, QInputDialog, QMessageBox
from PyQt5.QtCore import QTimer, Qt, QPoint
from PyQt5.QtGui import QImage, QPixmap
from pydantic import BaseModel
import google.generativeai as genai
import threading

# 1.2 將路徑退回到專案根目錄，跨資料夾 import Insta_OpenCV，並加入 sys.path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from Insta_OpenCV.controller.insta_worker import InstaWorker                # 初始化 Insta 相機
from Insta_OpenCV.utils.frame_receiver import FrameReceiver                 # RTMP 影像接收
from Insta_OpenCV.services.image_processing import ImageProcessingService   # 影像處理服務
from Insta_OpenCV.services.system_monitor import get_global_system_monitor  # 系統資源監控服務

# 新增: 導入 OBS 串流捕獲模組
from obs_stream_capture import OBSStreamCapture                             # OBS 視頻捕獲
from obs_audio import OBSAudioCapture                                       # OBS 音頻捕獲

# 1.3 設定日誌
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


# ====================================================
# ========== Step 2 AI 與 Schema 定義 =============
# ====================================================

# 2.1 定義台灣身份證 Schema
class TaiwanIDCard(BaseModel):
    id_number: str
    name: str

# 2.2 Gemini client 初始化
def get_gemini_api_key():
    """獲取 Gemini API Key，優先從環境變數，否則彈出輸入對話框"""
    api_key = os.environ.get("GEMINI_API_KEY")
    
    if not api_key or api_key == "YOUR_API_KEY_HERE":
        # 彈出輸入對話框
        api_key, ok = QInputDialog.getText(
            None, 
            'Gemini API Key', 
            '請輸入你的 Gemini API Key:\n(也可設定環境變數 GEMINI_API_KEY)',
            text=""
        )
        
        if not ok or not api_key.strip():
            QMessageBox.warning(None, "警告", "未輸入 API Key，OCR 功能將無法使用！")
            return "YOUR_API_KEY_HERE"
        
        # 可選：將 API Key 設定到環境變數（僅限當前程序）
        os.environ["GEMINI_API_KEY"] = api_key.strip()
    
    return api_key.strip()

# 先用環境變數或預設值初始化，之後在 main() 中會重新設定
genai.configure(api_key=os.environ.get("GEMINI_API_KEY", "YOUR_API_KEY_HERE"))
model = genai.GenerativeModel('gemini-1.5-pro')

# 備註: 
# 這個函式被呼叫後將會執行以下步驟
# 1. 根據是否選擇 ROI，決定使用全畫面或 ROI 區域進行 OCR
# 2. 使用 ImageProcessingService 進行品質增強
# 3. 呼叫 Gemini OCR API 進行辨識: 
# Gemini API Key 請設定在系統環境變數中，或直接替換上方 YOUR_API_KEY_HERE
# 可於 cmd 中執行: setx GEMINI_API_KEY "你的API金鑰"
# 或在 Linux/MacOS 終端機中執行: export GEMINI_API_KEY="你的API金鑰"
# 詳細請參考官方文件: https://developers.generativeai.google/products/gemini/get-started


# ====================================================
# ========== Step 3 ROI 預覽視窗類別 =============
# ====================================================

# 定義 ROI 預覽視窗，創建一個新的 QWidget 類別
class ROIPreviewWindow(QWidget):
    
    # 3.1 初始化視窗與元件
    def __init__(self):
        super().__init__()                                              # 呼叫父類別初始化
        self.setWindowTitle('ROI 高畫質預覽')                            # 設定視窗標題
        self.setFixedSize(600, 400)                                     # 固定視窗大小
        self.move(850, 100)                                             # 移動視窗位置

        self.preview_label = QLabel()                                   # 預覽影像標籤
        self.preview_label.setAlignment(Qt.AlignCenter)                 # 置中對齊
        self.preview_label.setStyleSheet("border: 2px solid blue; background-color: #f0f0f0;")  # 設定樣式
        self.preview_label.setText("選擇ROI區域後\n將顯示高畫質預覽")                             # 預設文字
        
        self.info_label = QLabel("尚未選擇區域")                         # 資訊標籤
        self.info_label.setAlignment(Qt.AlignCenter)                    # 置中對齊
        self.info_label.setStyleSheet("color: #666; font-size: 12px;")  # 設定樣式
        
        layout = QVBoxLayout()                                          # 垂直佈局
        layout.addWidget(self.preview_label, 1)                         # 預覽標籤佔大部分空間
        layout.addWidget(self.info_label, 0)                            # 資訊標籤佔較小空間
        self.setLayout(layout)                                          # 設定佈局

    # 3.2 更新預覽影像
    def update_preview(self, roi_frame):
        
        # 更新預覽影像
        if roi_frame is not None and roi_frame.size > 0:
            img_service = ImageProcessingService()                              # 初始化影像處理服務
            pixmap = img_service.convert_frame_to_qt_format(roi_frame)          # 直接轉換原始 ROI 畫面
            self.preview_label.setPixmap(pixmap)                                # 設定預覽影像
            
            # 顯示原始 ROI 解析度資訊
            h, w = roi_frame.shape[:2]
            self.info_label.setText(f"原始 ROI 解析度: {w}x{h}")
        else:
            self.clear_preview()
    
    # 3.3 清除預覽
    def clear_preview(self):
        self.preview_label.clear()
        self.preview_label.setText("選擇ROI區域後\n將顯示高畫質預覽")
        self.info_label.setText("尚未選擇區域")


# ====================================================
# ========== Step 4 OCR 預覽視窗類別 =============
# ====================================================

# 定義 OCR 攝像頭主介面類別
class OCRCameraWidget(QWidget):
    
    # 4.1 初始化主介面
    def __init__(self):
        super().__init__()                                              # 呼叫父類別初始化
        self.setWindowTitle('Gemini OCR 身份證辨識')                     # 設定視窗標題
        self.setFixedSize(800, 980)                                     # 固定視窗大小 (增加音頻顯示空間)

        # 4.1.1 初始化影像處理服務與系統監控
        self.image_service = ImageProcessingService()
        logger.info(f"[OCR] Image processing service initialized, GPU: {self.image_service.gpu_processor.is_gpu_available()}")
        self.system_monitor = get_global_system_monitor()
        self.system_monitor.start_monitoring()
        logger.info("[OCR] System monitoring started")
        self.current_frame = None
        self.original_frame = None  # 保存原始未 resize 的畫面
        
        # 修改: 使用 OBS 串流捕獲替代原有的 receiver 和 worker
        self.obs_video_capture = None  # OBS 視頻捕獲器
        self.obs_audio_capture = None  # OBS 音頻捕獲器
        self.insta_worker = None       # Insta Worker (用於啟動串流)
        
        # 串流配置
        self.insta_ip = "192.168.1.188"
        self.rtmp_url = "rtmp://192.168.1.188:1935/live/preview"

        # 啟動系統資源紀錄執行緒
        self._csv_thread = threading.Thread(target=export_system_metrics_to_csv, args=(self.system_monitor,), daemon=True)
        self._csv_thread.start()

        # 4.1.2 初始化 ROI 選擇相關變數
        self.roi_start = None
        self.roi_end = None
        self.roi_selecting = False
        self.roi_selected = False
        self.display_scale = 1.0
        self.display_offset_x = 0
        self.display_offset_y = 0
        self.frame_count = 0

        # 4.1.3 初始化 UI 元件
        self._init_ui()                                                 # 初始化 UI 元件
        self.roi_preview = ROIPreviewWindow()                           # 初始化 ROI 預覽視窗
        self.roi_preview.hide()                                         # 預設隱藏
        self._start_frame_timer()                                       # 啟動影像更新定時器
        self._init_camera_system()                                      # 初始化攝像頭系統
        
    # 4.2 初始化 UI 元件
    def _init_ui(self):
       
        # 4.2.1 影像顯示標籤
        self.image_label = QLabel()                                     # 影像顯示標籤
        self.image_label.setAlignment(Qt.AlignCenter)                   # 置中對齊
        self.image_label.setMinimumSize(780, 440)                       # 設定最小尺寸
        self.image_label.setStyleSheet("border: 2px solid gray;")       # 設定樣式
        self.image_label.setMouseTracking(True)                         # 啟用滑鼠追蹤
        self.image_label.installEventFilter(self)                       # 安裝事件過濾器
        
        # 4.2.2 控制按鈕
        self.roi_btn = QPushButton('選擇辨識區域 (拖拽框選)')             # ROI 選擇按鈕
        self.roi_btn.clicked.connect(self.toggle_roi_mode)              # 連接按鈕事件
        self.roi_btn.setMinimumHeight(40)                               # 設定最小高度
        self.clear_roi_btn = QPushButton('清除選擇區域')                 # 清除 ROI 按鈕
        self.clear_roi_btn.clicked.connect(self.clear_roi)              # 連接按鈕事件
        self.clear_roi_btn.setMinimumHeight(40)                         # 設定最小高度
        self.capture_btn = QPushButton('拍照並辨識')                     # 拍照並辨識按鈕
        self.capture_btn.clicked.connect(self.capture_and_ocr)          # 連接按鈕事件
        self.capture_btn.setMinimumHeight(40)                           # 設定最小高度

        # 4.2.3 結果顯示
        self.result_text = QTextEdit()
        self.result_text.setReadOnly(True)
        self.result_text.setMinimumHeight(200)
        
        # 4.2.4 狀態與資源監控顯示
        self.status_label = QLabel("系統狀態: 初始化中...")
        self.status_label.setStyleSheet("color: #666; font-size: 10px; padding: 5px;")
        self.resource_label = QLabel("系統資源: 監控中...")
        self.resource_label.setStyleSheet("color: #333; font-size: 10px; padding: 5px; background-color: #f0f0f0;")
        self.resource_label.setWordWrap(True)
        
        # 4.2.5 音頻顯示區域
        self.audio_label = QLabel("音頻狀態: 初始化中...")
        self.audio_label.setStyleSheet("color: #333; font-size: 10px; padding: 5px; background-color: #e8f4f8;")
        self.audio_label.setWordWrap(True)
        
        # 音量條顯示
        self.volume_bar = QLabel()
        self.volume_bar.setFixedHeight(20)
        self.volume_bar.setStyleSheet("border: 1px solid #ccc; background-color: #f0f0f0;")
        self.volume_bar.setText("音量: --")
        self.volume_bar.setAlignment(Qt.AlignCenter)
        
        # 4.2.6 佈局
        button_layout = QHBoxLayout()                                   # 水平佈局
        button_layout.addWidget(self.roi_btn)                           # 加入 ROI 按鈕
        button_layout.addWidget(self.clear_roi_btn)                     # 加入清除 ROI 按鈕
        button_layout.addWidget(self.capture_btn)                       # 加入拍照辨識按鈕
        
        # 音頻顯示佈局
        audio_layout = QHBoxLayout()
        audio_layout.addWidget(QLabel("🔊 音量:"))
        audio_layout.addWidget(self.volume_bar, 1)
        
        layout = QVBoxLayout()                                          # 垂直佈局
        layout.addWidget(self.image_label, 1)                           # 影像標籤
        layout.addLayout(button_layout, 0)                              # 按鈕佈局
        layout.addWidget(self.result_text, 0)                           # 結果顯示
        layout.addWidget(self.status_label, 0)                          # 狀態顯示
        layout.addWidget(self.resource_label, 0)                        # 資源監控顯示
        layout.addLayout(audio_layout, 0)                               # 音頻顯示佈局
        layout.addWidget(self.audio_label, 0)                           # 音頻詳細資訊
        self.setLayout(layout)                                          # 設定佈局
    
    
    # 4.3 初始化攝像頭系統 (修改為使用 OBS)
    def _init_camera_system(self):
        
        try:
            logger.info("[OCR] Initializing camera system with OBS...")
            
            # 4.3.1 初始化 InstaWorker (用於啟動 RTMP 串流)
            self.insta_worker = InstaWorker(ip_address=self.insta_ip)
            ready_event = self.insta_worker.start_preview_all()
            
            # 4.3.2 等待 InstaWorker 就緒
            logger.info("[OCR] Waiting for Insta worker to be ready...")
            ready_event.wait()
            logger.info("[OCR] Insta worker ready! Waiting for stream stabilization...")
            time.sleep(5)  # 等待串流穩定
            
            # 4.3.3 初始化 OBS 視頻捕獲器 - 設置顯示縮放
            logger.info("[OCR] Starting OBS video capture...")
            self.obs_video_capture = OBSStreamCapture(
                stream_url=self.rtmp_url,
                capture_audio=False,  # 音頻單獨處理
                display_scale=0.33,   # 直接在 OBS 層縮放到 1/3
                max_display_width=1280,
                max_display_height=720
            )
            
            if not self.obs_video_capture.start():
                logger.error("[OCR] Failed to start OBS video capture")
                raise Exception("OBS video capture startup failed")
            
            logger.info("[OCR] OBS video capture started successfully")
            
            # 4.3.4 初始化 OBS 音頻捕獲器 (可選)
            try:
                self.obs_audio_capture = OBSAudioCapture(stream_url=self.rtmp_url)
                if self.obs_audio_capture.start_capture():
                    logger.info("[OCR] OBS audio capture started successfully")
                else:
                    logger.warning("[OCR] OBS audio capture failed to start")
            except Exception as e:
                logger.warning(f"[OCR] Audio capture initialization failed: {e}")
                self.obs_audio_capture = None
            
            # 初始化當前幀
            self.current_frame = None
            
            logger.info("[OCR] Camera system with OBS initialized successfully")
            self.status_label.setText("系統狀態: OBS 串流已連接，GPU加速已啟用")
            
        except Exception as e:
            logger.error(f"[OCR] Failed to initialize camera system: {e}")
            self.status_label.setText(f"系統狀態: 攝像頭初始化失敗 - {e}")
            
            # 清理資源
            if self.obs_video_capture:
                self.obs_video_capture.stop()
                self.obs_video_capture = None
            if self.obs_audio_capture:
                self.obs_audio_capture.stop_capture()
                self.obs_audio_capture = None
            if self.insta_worker:
                self.insta_worker.stop_all()
                self.insta_worker = None
    
    # 4.4 FPS 定時器與影像更新
    def _start_frame_timer(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(33)  # 30 FPS (1000ms/30 ≈ 33.33ms)
        
        # 啟動額外的低延遲定時器來清理舊幀
        self.cleanup_timer = QTimer()
        self.cleanup_timer.timeout.connect(self._cleanup_old_frames)
        self.cleanup_timer.start(16)  # 每16ms清理一次舊幀
        
    def _cleanup_old_frames(self):
        """清理舊幀以減少延遲 (修改為使用 OBS，非阻塞方式)"""
        if self.obs_video_capture:
            # 獲取最新幀，OBS 捕獲器內部已處理幀緩衝
            # 這裡只是檢查是否有新幀，不進行耗時的處理
            latest_frame = self.obs_video_capture.get_latest_frame()
            if latest_frame is not None and self.original_frame is None:
                # 只在沒有原始畫面時才設置，避免頻繁更新
                self.original_frame = latest_frame
        
    # 備註，影像更新邏輯:
    # 1. 從 FrameReceiver 獲取最新幀
    # 2. 使用 ImageProcessingService 處理影像顯示
    # 3. 繪製 ROI 覆蓋層
    # 假如拉取影像速度跟不上更新頻率，會跳過幀以保持 UI 流暢
    
    
    # 4.5 影像更新與顯示 (使用 OBS 預縮放畫面)
    def update_frame(self):
        
        # 4.5.1 檢查 OBS 捕獲器是否已初始化
        if self.obs_video_capture is None:
            return
            
        # 4.5.2 從 OBS 捕獲器獲取預縮放的顯示畫面
        display_frame = self.obs_video_capture.get_latest_frame()  # 已經是縮放後的畫面
        original_frame = self.obs_video_capture.get_original_frame()  # 原始高解析度畫面
                
        if display_frame is None:
            return
        
        # 4.5.2.1 保存兩種畫面
        if original_frame is not None:
            self.original_frame = original_frame.copy()
        
        self.current_frame = display_frame  # 使用預縮放的畫面進行顯示
        self.frame_count += 1
        
        # 4.5.3 使用 ImageProcessingService 處理影像顯示
        if self.current_frame is not None:
            label_width = self.image_label.width()                          # 取得標籤寬度
            label_height = self.image_label.height()                        # 取得標籤高度
            
            display_frame, scale, offset_x, offset_y = self.image_service.prepare_frame_for_display(
                self.current_frame, label_width, label_height
            )                                                               # 處理影像以適應標籤尺寸
            
            if display_frame is None:
                return
            
            # 4.5.4 儲存顯示參數供 ROI 計算使用
            self.display_scale = scale                                      # 縮放比例
            self.display_offset_x = offset_x                                # X 軸偏移
            self.display_offset_y = offset_y                                # Y 軸偏移

            # 4.5.5 繪製 ROI 覆蓋層
            if self.roi_selected and self.roi_start and self.roi_end:
                roi_coords = self._get_roi_display_coords()
                if roi_coords:
                    display_frame = self.image_service.draw_roi_overlay(
                        display_frame, roi_coords, color=(0, 255, 0), label="OCR Region"
                    )
            elif self.roi_selecting and self.roi_start and self.roi_end:
                roi_coords = self._get_roi_display_coords()
                if roi_coords:
                    display_frame = self.image_service.draw_roi_overlay(
                        display_frame, roi_coords, color=(255, 255, 0), thickness=2, label=""
                    )
            
            # 4.5.6 轉換為 Qt 格式並顯示
            pixmap = self.image_service.convert_frame_to_qt_format(display_frame)
            self.image_label.setPixmap(pixmap)
            
            # 保存處理後的顯示畫面
            self.display_frame = display_frame

        # 4.5.7 更新 OBS 和系統狀態（每200幀一次，降低頻率）
        if self.frame_count % 200 == 0:
            try:
                # 獲取 OBS 統計信息
                obs_stats = self.obs_video_capture.get_stats()
                fps_info = f"FPS: {obs_stats.get('fps', 0):.1f}"
                
                # 獲取音頻信息並更新顯示
                audio_info = ""
                if self.obs_audio_capture:
                    audio_level = self.obs_audio_capture.get_audio_level()
                    audio_info = f" | Audio: {audio_level:.2f}"
                    self._update_audio_display(audio_level)
                else:
                    self._update_audio_display(0.0)
                
                self.status_label.setText(f"系統狀態: OBS串流 {fps_info}{audio_info} | GPU加速已啟用")
                
                # 系統資源信息
                resource_summary = self.system_monitor.get_resource_summary()
                self.resource_label.setText(f"系統資源: {resource_summary}")
                
            except Exception as e:
                logger.warning(f"[OCR] Error updating status: {e}")
        
        # 音頻顯示更新 (每30幀一次，更頻繁以確保音頻可視化流暢)
        elif self.frame_count % 30 == 0 and self.obs_audio_capture:
            try:
                audio_level = self.obs_audio_capture.get_audio_level()
                self._update_audio_display(audio_level)
            except Exception as e:
                logger.warning(f"[Audio] Error updating audio display: {e}")
    
    # 4.5.8 更新音頻顯示
    def _update_audio_display(self, audio_level):
        """更新音頻顯示區域"""
        try:
            # 音量條可視化 (0-100%)
            volume_percent = min(100, max(0, audio_level * 100))
            bar_width = int(volume_percent * 2)  # 最大200像素寬度
            
            # 根據音量大小設置顏色
            if volume_percent > 80:
                color = "#ff4444"  # 紅色 - 過大
            elif volume_percent > 50:
                color = "#ffaa00"  # 橙色 - 適中偏大
            elif volume_percent > 20:
                color = "#44ff44"  # 綠色 - 適中
            else:
                color = "#aaaaaa"  # 灰色 - 過小
            
            # 更新音量條樣式
            self.volume_bar.setStyleSheet(f"""
                QLabel {{
                    border: 1px solid #ccc;
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                        stop:0 {color}, stop:{volume_percent/100:.2f} {color},
                        stop:{volume_percent/100:.2f} #f0f0f0, stop:1 #f0f0f0);
                    color: black;
                    font-weight: bold;
                }}
            """)
            self.volume_bar.setText(f"音量: {volume_percent:.1f}%")
            
            # 更新音頻詳細資訊
            if self.obs_audio_capture:
                audio_info = self.obs_audio_capture.get_audio_info()
                self.audio_label.setText(
                    f"音頻詳情: 採樣率: {audio_info.get('sample_rate', 'N/A')}Hz | "
                    f"聲道: {audio_info.get('channels', 'N/A')} | "
                    f"位深: {audio_info.get('bit_depth', 'N/A')}bit | "
                    f"音量: {volume_percent:.1f}%"
                )
            else:
                self.audio_label.setText("音頻詳情: 音頻捕獲未啟用")
                
        except Exception as e:
            logger.warning(f"[Audio] Error updating audio display: {e}")
            self.audio_label.setText(f"音頻狀態: 更新錯誤 - {e}")
    
    
    # 4.6 ROI 選擇相關方法
    def _get_roi_display_coords(self):
        
        if not (self.roi_start and self.roi_end):
            return None
        
        x1 = min(self.roi_start.x(), self.roi_end.x())                  # 左上角 X
        y1 = min(self.roi_start.y(), self.roi_end.y())                  # 左上角 Y
        x2 = max(self.roi_start.x(), self.roi_end.x())                  # 右下角 X
        y2 = max(self.roi_start.y(), self.roi_end.y())                  # 右下角 Y

        return (x1, y1, x2, y2)
    
    
    # 4.7 事件過濾器處理滑鼠事件
    def eventFilter(self, obj, event):
        
        # 4.7.1 僅處理影像標籤上的滑鼠事件，且需有影像與 OBS 捕獲器
        if obj == self.image_label and self.current_frame is not None and self.obs_video_capture is not None:
            
            # 4.7.2 處理滑鼠左鍵按下事件，開始 ROI 選擇
            if event.type() == event.MouseButtonPress:      # 處理滑鼠按下事件
                if event.button() == Qt.LeftButton:
                    mouse_pos = self._convert_mouse_coords(event.pos())
                    if mouse_pos:
                        self.roi_start = mouse_pos
                        self.roi_selecting = True
                        self.roi_selected = False
                        logger.info(f"[ROI] Start selection at: {mouse_pos.x()}, {mouse_pos.y()}")
                    return True
                    
            # 4.7.3 處理滑鼠移動事件，動態更新 ROI 區域
            elif event.type() == event.MouseMove:           # 處理滑鼠移動事件
                if self.roi_selecting and self.roi_start:
                    mouse_pos = self._convert_mouse_coords(event.pos())
                    if mouse_pos:
                        self.roi_end = mouse_pos
                        # 降低 ROI 預覽更新頻率，避免卡頓
                        if self.frame_count % 3 == 0:  # 每3幀更新一次
                            self._update_roi_preview()
                    return True
                    
            # 4.7.4 處理滑鼠左鍵釋放事件，完成 ROI 選擇
            elif event.type() == event.MouseButtonRelease:  # 處理滑鼠釋放事件
                if event.button() == Qt.LeftButton and self.roi_selecting:
                    mouse_pos = self._convert_mouse_coords(event.pos())
                    if mouse_pos:
                        self.roi_end = mouse_pos
                        self.roi_selecting = False
                        self.roi_selected = True
                        self.roi_preview.show()
                        self._update_roi_preview()
                        logger.info(f"[ROI] Selection completed: ({self.roi_start.x()}, {self.roi_start.y()}) to ({self.roi_end.x()}, {self.roi_end.y()})")
                    return True
        
        # 4.7.5 其他事件交由父類別處理
        return super().eventFilter(obj, event)
    
    
    # 4.8 滑鼠坐標轉換
    def _convert_mouse_coords(self, mouse_pos):
        
        if self.current_frame is None:                                          # 沒有當前影像就回傳 None
            return None
            
        img_h, img_w = self.current_frame.shape[:2]                             # 取得影像尺寸
        
        return self.image_service.convert_mouse_to_image_coords(                # 轉換坐標
            mouse_pos, img_w, img_h,
            self.display_scale, self.display_offset_x, self.display_offset_y
        )
    
    
    # 4.9 更新 ROI 預覽視窗 (修改為使用原始畫面)
    def _update_roi_preview(self):
        
        if self.roi_start and self.roi_end and self.original_frame is not None:  # 使用原始畫面而非縮放後的畫面
            roi_coords = self._get_roi_display_coords()                         # 取得 ROI 坐標
            if roi_coords:                                                      # 確保 ROI 坐標有效
                # 將顯示坐標轉換為原始圖像坐標
                orig_roi_coords = self._convert_display_to_original_coords(roi_coords)
                if orig_roi_coords:
                    # 從原始畫面提取 ROI（未經 resize）
                    x1, y1, x2, y2 = orig_roi_coords
                    roi_frame = self.original_frame[y1:y2, x1:x2]
                    logger.info(f"[ROI] Extracted original ROI: {roi_frame.shape} from original: {self.original_frame.shape}")
                    self.roi_preview.update_preview(roi_frame)                      # 更新預覽視窗
    
    def _convert_display_to_original_coords(self, roi_coords):
        """將顯示坐標轉換為原始圖像坐標"""
        if self.original_frame is None:
            return None
        
        x1, y1, x2, y2 = roi_coords
        
        # 獲取原始圖像和當前顯示圖像的尺寸
        orig_h, orig_w = self.original_frame.shape[:2]
        curr_h, curr_w = self.current_frame.shape[:2]
        
        # 計算縮放比例
        scale_x = orig_w / curr_w
        scale_y = orig_h / curr_h
        
        # 轉換坐標到原始圖像
        orig_x1 = int(x1 * scale_x)
        orig_y1 = int(y1 * scale_y)
        orig_x2 = int(x2 * scale_x)
        orig_y2 = int(y2 * scale_y)
        
        # 確保坐標在有效範圍內
        orig_x1 = max(0, min(orig_x1, orig_w - 1))
        orig_y1 = max(0, min(orig_y1, orig_h - 1))
        orig_x2 = max(0, min(orig_x2, orig_w - 1))
        orig_y2 = max(0, min(orig_y2, orig_h - 1))
        
        return (orig_x1, orig_y1, orig_x2, orig_y2)
    
    
    # 4.10 切換 ROI 選擇模式
    def toggle_roi_mode(self):
        
        if self.roi_selected:                                                   # 如果已選擇 ROI，則清除        
            self.clear_roi()                                                    # 清除 ROI 選擇
        else:
            self.roi_btn.setText('選擇辨識區域 (拖拽框選中...)')                  # 更新按鈕文字
            logger.info("[ROI] ROI selection mode enabled")                     # 記錄日誌
    
    
    # 4.11 清除 ROI 選擇
    def clear_roi(self):
        
        self.roi_start = None                                                   # 清除起點
        self.roi_end = None                                                     # 清除終點
        self.roi_selecting = False                                              # 清除選擇狀態
        self.roi_selected = False                                               # 清除已選擇狀態
        self.roi_btn.setText('選擇辨識區域 (拖拽框選)')                           # 重置按鈕文字
        self.roi_preview.hide()                                                 # 隱藏預覽視窗
        self.roi_preview.clear_preview()                                        # 清除預覽內容
        logger.info("[ROI] ROI selection cleared")                              # 記錄日誌
    
    # ====================================================
    # ========== Step 4.12 拍照並進行 OCR 辨識 =============
    # ====================================================
    
    # 備註: 
    # 這個函式被呼叫後將會執行以下步驟
    # 1. 根據是否選擇 ROI，決定使用全畫面或 ROI 區域進行 OCR
    # 2. 使用 ImageProcessingService 進行品質增強
    # 3. 呼叫 Gemini OCR API 進行辨識
    # 4. 根據辨識結果與白名單進行許可/異常判斷
    # 5. 顯示辨識結果與判斷狀態
    
    def capture_and_ocr(self):
        
        # 4.12.0 確保有當前影像
        if self.current_frame is None:                                          # 確保有當前影像
            logger.warning("[OCR] No frame available for capture")              # 記錄警告
            return

        # 4.12.1 許可人員白名單，可改為讀取外部檔案
        whitelist = [
            {"id_number": "A123456789", "name": "王小明"},
            {"id_number": "B987654321", "name": "李小華"},
        ]

        # 4.12.2 定義許可判斷函式，此函式為內部函式，不會被外部呼叫
        def is_permitted(ocr_result, whitelist):
            for person in whitelist:
                if (ocr_result.get("id_number") == person["id_number"] and      # 比對身分證號
                    ocr_result.get("name") == person["name"]):                  # 比對姓名
                    return True
            return False

        try:
            # 4.12.3 決定處理區域
            if self.roi_selected and self.roi_start and self.roi_end:           # 使用 ROI 區域
                roi_coords = self._get_roi_display_coords()                     # 取得 ROI 坐標
                # 使用原始畫面提取 ROI，確保 OCR 也使用高解析度
                orig_roi_coords = self._convert_display_to_original_coords(roi_coords)
                if orig_roi_coords:
                    x1, y1, x2, y2 = orig_roi_coords
                    processed_frame = self.original_frame[y1:y2, x1:x2]         # 從原始畫面提取 ROI
                    logger.info(f"[OCR] Using original ROI region for OCR: {processed_frame.shape}")
                else:
                    logger.error("[OCR] Failed to convert ROI coordinates")
                    return
            else:
                processed_frame = self.original_frame if self.original_frame is not None else self.current_frame  # 使用原始全畫面
                logger.info("[OCR] Using original full frame for OCR")           # 記錄日誌

            if processed_frame is None:
                self.result_text.setText("錯誤：無法獲取有效的影像區域")          # 顯示錯誤訊息
                return

            # 4.12.4 品質增強
            processed_frame = self.image_service.enhance_frame_quality(processed_frame)     # 呼叫 image_service 進行品質增強

            # 4.12.5 儲存暫存圖片
            img_path = 'temp_capture.jpg'
            cv2.imwrite(img_path, processed_frame)

            # 4.12.6 讀取圖片為位元組並呼叫 Gemini OCR
            with open(img_path, 'rb') as f:
                image_bytes = f.read()

            logger.info(f"[OCR] Processing image with size: {processed_frame.shape}")
            img_part = {
                "mime_type": "image/jpeg",
                "data": image_bytes
            }

            resp = model.generate_content(                                                  # 呼叫 Gemini OCR API
                [img_part, "這是一張台灣身份證照片，請回傳身分證號(id_number)與姓名(name)。"],  # 提供圖片與提示語
                generation_config=genai.GenerationConfig(                                   # 設定回應格式
                    response_mime_type="application/json",                                  # 回傳 JSON 格式
                    response_schema=TaiwanIDCard                                            # 使用定義的 Schema
                )
            )
            
            # 備註:
            # Gemini OCR API 提供可解析的 JSON 回應，包含辨識出的身分證號與姓名
            # 回應格式範例: {"id_number": "A123456789", "name": "王小明"}
            # 若回應無法解析，會顯示錯誤訊息並結束

            # 4.12.7 處理回應
            ocr_result = None
            result_text = ""
            if resp.candidates and resp.candidates[0].content and resp.candidates[0].content.parts: # 檢查回應結構
                part = resp.candidates[0].content.parts[0]                                          # 取得第一個部分
                
                if part.function_call:                                                              # 檢查是否為函式呼叫回應
                    parsed_args = dict(part.function_call.args)                                     # 解析函式參數
                    ocr_result = parsed_args                                                        # 儲存 OCR 結果
                    result_text = json.dumps(parsed_args, ensure_ascii=False, indent=2)             # 格式化顯示結果
                
                elif resp.text:                                                                     # 有文字回應但無法解析為函式呼叫
                    try:
                        parsed_args = json.loads(resp.text)                                         # 嘗試解析 JSON
                        ocr_result = parsed_args                                                    # 儲存 OCR 結果
                        result_text = resp.text                                                     # 顯示原始文字回應
                    except json.JSONDecodeError:                                                    # 解析失敗
                        self.result_text.setText(f"無法解析OCR回應。\n\n原始回應:\n{resp.text}")
                        return
                else:
                    self.result_text.setText("無法解析OCR回應，且沒有收到文字回應。")
                    return
            else:
                error_message = "無法解析OCR回應。請再試一次。"
                if resp.text:
                    error_message += f"\n\n原始回應:\n{resp.text}"
                self.result_text.setText(error_message)
                return

            # 4.12.8 許可/異常判斷
            if ocr_result:
                permitted = is_permitted(ocr_result, whitelist)
                status_str = "✅ 許可人員" if permitted else "❌ 異常人員"
                result_text += f"\n---\nPython物件：\n身分證號: {ocr_result.get('id_number', 'N/A')}\n姓名: {ocr_result.get('name', 'N/A')}\n身份判斷：{status_str}"
                self.result_text.setText(result_text)
            else:
                self.result_text.setText("OCR 結果無法判斷許可/異常人員。")

        except Exception as e:
            logger.error(f"[OCR] Error during OCR processing: {e}")
            self.result_text.setText(f"OCR 發生錯誤：{e}")
        finally:
            try:
                os.remove(img_path)
            except:
                pass
    
    
    # 4.13 關閉事件處理 (修改為清理 OBS 資源)
    def closeEvent(self, event):
        """關閉事件處理"""
        logger.info('[OCR] Shutting down...')
        self.timer.stop()
        self.cleanup_timer.stop()
        
        # 清理 OBS 資源
        if self.obs_video_capture:
            self.obs_video_capture.stop()
            self.obs_video_capture = None
        
        if self.obs_audio_capture:
            self.obs_audio_capture.stop_capture()
            self.obs_audio_capture = None
        
        # 清理 Insta Worker
        if self.insta_worker:
            self.insta_worker.stop_all()
            self.insta_worker = None
        
        # 清理系統監控
        if hasattr(self, 'system_monitor'):
            self.system_monitor.stop_monitoring()
        
        # 關閉 ROI 預覽窗口
        if hasattr(self, 'roi_preview'):
            self.roi_preview.close()
        
        logger.info('[OCR] Shutdown complete.')
        event.accept()

# =============================================================
# ================= Step 5 輸出紀錄數值變化 csv ==================
# =============================================================

# 在此紀錄下 GPU 使用狀況、CPU 使用率、記憶體和畫面 FPS 變化等系統資源數值變化
# 可選擇性將這些數值輸出到 CSV 檔案中，方便後續分析
# 例如: 在 get_global_system_monitor() 中加入 CSV 輸出功能
# 或在 OCRCameraWidget 中定期將 self.system_monitor.get_resource_summary() 寫入 CSV
# 這部分可依需求自行擴展

def export_system_metrics_to_csv(system_monitor, csv_path='system_metrics.csv'):
    import csv
    import logging
    logger = logging.getLogger(__name__)
    
    while True:
        try:
            with open(csv_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                # 若檔案剛建立，寫入標題
                if file.tell() == 0:
                    writer.writerow(['Timestamp', 'CPU_Usage(%)', 'Memory_Usage(%)', 'Memory_Used(GB)', 'GPU_Usage(%)', 'GPU_Memory_Used(MB)', 'GPU_Temp(C)', 'CPU_Temp(C)', 'FPS', 'Frame_Count', 'Latency(ms)'])
                
                # 直接獲取 SystemMetrics 對象
                metrics = system_monitor.get_metrics()
                timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
                
                # 嘗試獲取FPS數據（從全域或其他來源）
                fps = 0.0
                frame_count = 0
                latency = 0.0
                try:
                    # 嘗試從performance monitor獲取FPS數據
                    from Insta_OpenCV.services.performance_monitor import get_global_performance_monitor
                    perf_monitor = get_global_performance_monitor()
                    perf_metrics = perf_monitor.get_metrics()
                    fps = perf_metrics.fps
                    frame_count = perf_metrics.frame_count
                    latency = perf_metrics.latency
                except Exception as e:
                    logger.debug(f"[SystemMetrics] Could not get FPS data: {e}")
                
                writer.writerow([
                    timestamp,
                    round(metrics.cpu_percent, 2),
                    round(metrics.memory_percent, 2),
                    round(metrics.memory_used_gb, 2),
                    round(metrics.gpu_percent, 2),
                    round(metrics.gpu_memory_used_mb, 2),
                    round(metrics.gpu_temp, 2),
                    round(metrics.cpu_temp, 2),
                    round(fps, 2),
                    frame_count,
                    round(latency, 2)
                ])
                
                logger.debug(f"[SystemMetrics] Recorded: CPU={metrics.cpu_percent:.1f}%, GPU={metrics.gpu_percent:.1f}%, Memory={metrics.memory_percent:.1f}%, FPS={fps:.1f}")
                
        except Exception as e:
            # 記錄錯誤但不中斷執行緒
            logger.warning(f"[SystemMetrics] Failed to write metrics: {e}")
        time.sleep(5)  # 每5秒紀錄一次


# ====================================================
# ================= Step 6 主程式 ==================
# ====================================================

def main():
    
    app = QApplication(sys.argv)                                 # 初始化 QApplication
    
    # 在 QApplication 建立後檢查並設定 API Key
    try:
        api_key = get_gemini_api_key()
        genai.configure(api_key=api_key)
        global model
        model = genai.GenerativeModel('gemini-1.5-pro')
    except Exception as e:
        QMessageBox.critical(None, "錯誤", f"API Key 設定失敗: {e}")
        return
    
    try:
        window = OCRCameraWidget()                               # 初始化主視窗
        window.show()                                            # 顯示主視窗
        sys.exit(app.exec_())                                    # 執行應用程式主迴圈
    except Exception as e:                                       # 捕捉所有異常
        logger.error(f"[Main] Application error: {e}")           # 記錄錯誤
        sys.exit(1)                                              # 非正常退出

if __name__ == '__main__':
    main()