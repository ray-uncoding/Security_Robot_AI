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
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QTextEdit
from PyQt5.QtCore import QTimer, Qt, QPoint
from PyQt5.QtGui import QImage, QPixmap
from pydantic import BaseModel
import google.generativeai as genai

# 1.2 將路徑退回到專案根目錄，跨資料夾 import Insta_OpenCV，並加入 sys.path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from Insta_OpenCV.controller.insta_worker import InstaWorker                # 初始化 Insta 相機
from Insta_OpenCV.utils.frame_receiver import FrameReceiver                 # RTMP 影像接收
from Insta_OpenCV.services.image_processing import ImageProcessingService   # 影像處理服務
from Insta_OpenCV.services.system_monitor import get_global_system_monitor  # 系統資源監控服務

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
genai.configure(api_key=os.environ.get("GEMINI_API_KEY", "YOUR_API_KEY_HERE"))
model = genai.GenerativeModel('gemini-1.5-pro')

# 備註: 
# 這個函式被呼叫後將會執行以下步驟
# # 1. 根據是否選擇 ROI，決定使用全畫面或 ROI 區域進行 OCR
# # 2. 使用 ImageProcessingService 進行品質增強
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
            
            preview_w = self.preview_label.width() - 10                 # 留點邊距
            preview_h = self.preview_label.height() - 10                # 留點邊距
            h, w = roi_frame.shape[:2]                                  # 取得 ROI 影像尺寸
            scale = min(preview_w / w, preview_h / h)                   # 計算縮放比例
            new_w = int(w * scale)                                      # 計算新寬度
            new_h = int(h * scale)                                      # 計算新高度
            
            img_service = ImageProcessingService()                                              # 初始化影像處理服務
            resized_roi = cv2.resize(roi_frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR) # 調整大小
            
            pixmap = img_service.convert_frame_to_qt_format(resized_roi)                        # 轉換為 Qt 格式
            self.preview_label.setPixmap(pixmap)                                                # 設定預覽影像
            self.info_label.setText(f"ROI解析度: {w}x{h} | 預覽: {new_w}x{new_h}")               # 顯示解析度資訊   
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
        self.setFixedSize(800, 900)                                     # 固定視窗大小
        
        # 4.1.1 初始化影像處理服務與系統監控
        self.image_service = ImageProcessingService()
        logger.info(f"[OCR] Image processing service initialized, GPU: {self.image_service.gpu_processor.is_gpu_available()}")
        self.system_monitor = get_global_system_monitor()
        self.system_monitor.start_monitoring()
        logger.info("[OCR] System monitoring started")
        self.current_frame = None
        self.receiver = None
        self.worker = None
        
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
        
        # 4.2.5 佈局
        button_layout = QHBoxLayout()                                   # 水平佈局
        button_layout.addWidget(self.roi_btn)                           # 加入 ROI 按鈕
        button_layout.addWidget(self.clear_roi_btn)                     # 加入清除 ROI 按鈕
        button_layout.addWidget(self.capture_btn)                       # 加入拍照辨識按鈕
        layout = QVBoxLayout()                                          # 垂直佈局
        layout.addWidget(self.image_label, 1)                           # 影像標籤
        layout.addLayout(button_layout, 0)                              # 按鈕佈局
        layout.addWidget(self.result_text, 0)                           # 結果顯示
        layout.addWidget(self.status_label, 0)                          # 狀態顯示
        layout.addWidget(self.resource_label, 0)                        # 資源監控顯示
        self.setLayout(layout)                                          # 設定佈局
    
    
    # 4.3 初始化攝像頭系統
    def _init_camera_system(self):
        
        try:
            logger.info("[OCR] Initializing camera system...")
            
            # 4.3.1 初始化 InstaWorker
            self.worker = InstaWorker(ip_address="192.168.1.188")             # Insta 相機預設 IP 為 192.168.1.188
            ready_event = self.worker.start_preview_all()                     # 啟動預覽並獲取就緒事件
            
            # 備註: 
            # 這個函式被呼叫後將會執行以下步驟
            # # 1. 根據是否選擇 ROI，決定使用全畫面或 ROI 區域進行 OCR
            # # 2. 使用 ImageProcessingService 進行品質增強
            # 3. 呼叫 Gemini OCR API 進行辨識:
            # InstaWorker 支援 GPU 加速，若系統有 NVIDIA GPU 且安裝了 CUDA 驅動，會自動啟用 GPU 模式
            # InstaWorker start_preview_all() 會返回一個 threading.Event 物件，並啟動獨立的心跳線程
            
            # 4.3.2 等待 InstaWorker 就緒
            logger.info("[OCR] Waiting for worker to be ready...")
            ready_event.wait()                                                # 阻塞直到相機就緒
            logger.info("[OCR] Worker ready! Waiting for stream stabilization...")
            time.sleep(5)                                                     # 等待流穩定
            
            # 4.3.3 初始化 FrameReceiver
            camera_rtmp_url = "rtmp://192.168.1.188:1935/live/preview"        # Insta 相機 RTMP 流地址
            self.receiver = FrameReceiver(stream_url=camera_rtmp_url)         # 從 RTMP 流接收影像
            self.receiver.start()                                             # 啟動接收器
            
            self.current_frame = None                                         # 初始化當前幀
            
            logger.info("[OCR] Camera system initialized successfully")
            self.status_label.setText("系統狀態: 攝像頭已連接，GPU加速已啟用")
            
        except Exception as e:
            logger.error(f"[OCR] Failed to initialize camera system: {e}")
            self.status_label.setText(f"系統狀態: 攝像頭初始化失敗 - {e}")
    
    # 4.4 FPS 定時器與影像更新
    def _start_frame_timer(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(16)  # 60 FPS
        
    # 備註，影像更新邏輯:
    # 1. 從 FrameReceiver 獲取最新幀
    # 2. 使用 ImageProcessingService 處理影像顯示
    # 3. 繪製 ROI 覆蓋層
    # 假如拉取影像速度跟不上更新頻率，會跳過幀以保持 UI 流暢
    
    
    # 4.5 影像更新與顯示
    def update_frame(self):
        
        # 4.5.1 檢查接收器是否已初始化
        if self.receiver is None:
            return
            
        # 4.5.2 從 FrameReceiver 獲取最新幀
        frame = self.receiver.get_latest_frame()
        if frame is None:
            return
        
        self.current_frame = frame
        self.frame_count += 1
        
        # 4.5.3 使用 ImageProcessingService 處理影像顯示
        label_width = self.image_label.width()                          # 取得標籤寬度
        label_height = self.image_label.height()                        # 取得標籤高度
        
        display_frame, scale, offset_x, offset_y = self.image_service.prepare_frame_for_display(
            frame, label_width, label_height
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

        # 4.5.7 更新 CUDA GPU 狀態（每100幀一次）
        if self.frame_count % 100 == 0:
            try:
                perf_summary = self.receiver.get_performance_summary()
                gpu_info = self.receiver.get_gpu_info()
                self.status_label.setText(f"系統狀態: {perf_summary} | GPU: {gpu_info['available']}")
                resource_summary = self.system_monitor.get_resource_summary()
                self.resource_label.setText(f"系統資源: {resource_summary}")
                
            except Exception as e:
                logger.warning(f"[OCR] Error updating status: {e}")
    
    
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
        
        # 4.7.1 僅處理影像標籤上的滑鼠事件，且需有影像與接收器
        if obj == self.image_label and self.current_frame is not None and self.receiver is not None:
            
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
    
    
    # 4.9 更新 ROI 預覽視窗
    def _update_roi_preview(self):
        
        if self.roi_start and self.roi_end and self.current_frame is not None:  # 確保 ROI 已選擇且有當前影像
            roi_coords = self._get_roi_display_coords()                         # 取得 ROI 坐標
            if roi_coords:                                                      # 確保 ROI 坐標有效
                roi_frame = self.image_service.extract_roi_from_frame(          # 擷取 ROI 影像
                    self.current_frame, roi_coords, self.display_scale          # 傳入縮放比例以獲取高解析度 ROI
                )
                self.roi_preview.update_preview(roi_frame)                      # 更新預覽視窗
    
    
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
                processed_frame = self.image_service.extract_roi_from_frame(    # 擷取 ROI 影像
                    self.current_frame, roi_coords, self.display_scale          # 傳入縮放比例以獲取高解析度 ROI
                )
                logger.info("[OCR] Using ROI region for OCR")                   # 記錄日誌
            else:
                processed_frame = self.current_frame                            # 使用全畫面
                logger.info("[OCR] Using full frame for OCR")                   # 記錄日誌

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
    
    
    # 4.13 關閉事件處理
    def closeEvent(self, event):
        """關閉事件處理"""
        logger.info('[OCR] Shutting down...')
        self.timer.stop()
        if self.receiver:
            self.receiver.stop()
        if self.worker:
            self.worker.stop_all()
        if hasattr(self, 'system_monitor'):
            self.system_monitor.stop_monitoring()
        if hasattr(self, 'roi_preview'):
            self.roi_preview.close()
        logger.info('[OCR] Shutdown complete.')
        event.accept()

# ====================================================
# ================= Step 5 主程式 ==================
# ====================================================

def main():
    
    app = QApplication(sys.argv)                                 # 初始化 QApplication
    try:
        window = OCRCameraWidget()                               # 初始化主視窗
        window.show()                                            # 顯示主視窗
        sys.exit(app.exec_())                                    # 執行應用程式主迴圈
    except Exception as e:                                       # 捕捉所有異常
        logger.error(f"[Main] Application error: {e}")           # 記錄錯誤
        sys.exit(1)                                              # 非正常退出

if __name__ == '__main__':
    main()
