"""重構後的 OCR 測試應用 - 使用清晰的架構分離"""
# ============================
# ========== Step 1 =============
# ====== 基本匯入與初始化 ========
# ============================

# 1.1 標準庫與第三方套件匯入
import sys
import os
import time
import json
import logging
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QTextEdit
from PyQt5.QtCore import QTimer, Qt, QPoint
from PyQt5.QtGui import QImage, QPixmap
from pydantic import BaseModel
import google.generativeai as genai


# 1.2 專案根目錄加入 sys.path，方便跨資料夾 import
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if project_root not in sys.path:
    sys.path.insert(0, project_root)


# 1.3 導入自家模組（影像、串流、系統監控）
from Insta_OpenCV.controller.insta_worker import InstaWorker
from Insta_OpenCV.utils.frame_receiver import FrameReceiver
from Insta_OpenCV.services.image_processing import ImageProcessingService
from Insta_OpenCV.services.system_monitor import get_global_system_monitor


# 1.4 設定日誌
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


# ============================
# ========== Step 2 =============
# ====== AI 與 Schema 定義 =======
# ============================

# 2.1 定義台灣身份證 Schema
class TaiwanIDCard(BaseModel):
    id_number: str
    name: str


# 2.2 Gemini client 初始化
genai.configure(api_key=os.environ.get("GEMINI_API_KEY", "YOUR_API_KEY_HERE"))
model = genai.GenerativeModel('gemini-1.5-pro')


# ============================
# ========== Step 3 =============
# ====== ROI 預覽視窗類別 ========
# ============================

class ROIPreviewWindow(QWidget):
    """ROI區域高畫質預覽視窗"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle('ROI 高畫質預覽')
        self.setFixedSize(600, 400)
        self.move(850, 100)
        # 3.1 預覽與資訊標籤
        self.preview_label = QLabel()
        self.preview_label.setAlignment(Qt.AlignCenter)
        self.preview_label.setStyleSheet("border: 2px solid blue; background-color: #f0f0f0;")
        self.preview_label.setText("選擇ROI區域後\n將顯示高畫質預覽")
        self.info_label = QLabel("尚未選擇區域")
        self.info_label.setAlignment(Qt.AlignCenter)
        self.info_label.setStyleSheet("color: #666; font-size: 12px;")
        layout = QVBoxLayout()
        layout.addWidget(self.preview_label, 1)
        layout.addWidget(self.info_label, 0)
        self.setLayout(layout)

    def update_preview(self, roi_frame):
        """更新ROI預覽影像"""
        if roi_frame is not None and roi_frame.size > 0:
            # 3.2 計算縮放與顯示
            preview_w = self.preview_label.width() - 10
            preview_h = self.preview_label.height() - 10
            h, w = roi_frame.shape[:2]
            scale = min(preview_w / w, preview_h / h)
            new_w = int(w * scale)
            new_h = int(h * scale)
            from Insta_OpenCV.services.image_processing import ImageProcessingService
            img_service = ImageProcessingService()
            import cv2
            resized_roi = cv2.resize(roi_frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
            pixmap = img_service.convert_frame_to_qt_format(resized_roi)
            self.preview_label.setPixmap(pixmap)
            self.info_label.setText(f"ROI解析度: {w}x{h} | 預覽: {new_w}x{new_h}")
        else:
            self.clear_preview()

    def clear_preview(self):
        """清除預覽"""
        self.preview_label.clear()
        self.preview_label.setText("選擇ROI區域後\n將顯示高畫質預覽")
        self.info_label.setText("尚未選擇區域")


# ============================
# ========== Step 4 =============
# ====== 主視窗/主流程類別 =======
# ============================

class OCRCameraWidget(QWidget):
    """OCR 攝像頭主介面"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Gemini OCR 身份證辨識 (重構版 - GPU加速)')
        self.setFixedSize(800, 900)
        # 4.1 初始化影像處理、系統監控、變數
        self.image_service = ImageProcessingService()
        logger.info(f"[OCR] Image processing service initialized, GPU: {self.image_service.gpu_processor.is_gpu_available()}")
        self.system_monitor = get_global_system_monitor()
        self.system_monitor.start_monitoring()
        logger.info("[OCR] System monitoring started")
        self.current_frame = None
        self.receiver = None
        self.worker = None
        # ROI 相關
        self.roi_start = None
        self.roi_end = None
        self.roi_selecting = False
        self.roi_selected = False
        self.display_scale = 1.0
        self.display_offset_x = 0
        self.display_offset_y = 0
        self.frame_count = 0
        # 4.2 初始化 UI
        self._init_ui()
        # 4.3 創建 ROI 預覽視窗
        self.roi_preview = ROIPreviewWindow()
        self.roi_preview.hide()
        # 4.4 啟動影像更新定時器
        self._start_frame_timer()
        # 4.5 初始化攝像頭系統
        self._init_camera_system()
        
    def _init_ui(self):
        """初始化使用者介面"""
        # 4.2.1 影像顯示標籤
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(780, 440)
        self.image_label.setStyleSheet("border: 2px solid gray;")
        self.image_label.setMouseTracking(True)
        self.image_label.installEventFilter(self)
        # 4.2.2 控制按鈕
        self.roi_btn = QPushButton('選擇辨識區域 (拖拽框選)')
        self.roi_btn.clicked.connect(self.toggle_roi_mode)
        self.roi_btn.setMinimumHeight(40)
        self.clear_roi_btn = QPushButton('清除選擇區域')
        self.clear_roi_btn.clicked.connect(self.clear_roi)
        self.clear_roi_btn.setMinimumHeight(40)
        self.capture_btn = QPushButton('拍照並辨識')
        self.capture_btn.clicked.connect(self.capture_and_ocr)
        self.capture_btn.setMinimumHeight(40)
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
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.roi_btn)
        button_layout.addWidget(self.clear_roi_btn)
        button_layout.addWidget(self.capture_btn)
        layout = QVBoxLayout()
        layout.addWidget(self.image_label, 1)
        layout.addLayout(button_layout, 0)
        layout.addWidget(self.result_text, 0)
        layout.addWidget(self.status_label, 0)
        layout.addWidget(self.resource_label, 0)
        self.setLayout(layout)
    
    def _init_camera_system(self):
        """初始化攝像頭系統"""
        try:
            logger.info("[OCR] Initializing camera system...")
            
            # 初始化 InstaWorker
            self.worker = InstaWorker(ip_address="192.168.1.188")
            ready_event = self.worker.start_preview_all()
            
            logger.info("[OCR] Waiting for worker to be ready...")
            ready_event.wait()
            logger.info("[OCR] Worker ready! Waiting for stream stabilization...")
            time.sleep(5)
            
            # 初始化 FrameReceiver
            camera_rtmp_url = "rtmp://192.168.1.188:1935/live/preview"
            self.receiver = FrameReceiver(stream_url=camera_rtmp_url)
            self.receiver.start()
            
            self.current_frame = None
            
            logger.info("[OCR] Camera system initialized successfully")
            self.status_label.setText("系統狀態: 攝像頭已連接，GPU加速已啟用")
            
        except Exception as e:
            logger.error(f"[OCR] Failed to initialize camera system: {e}")
            self.status_label.setText(f"系統狀態: 攝像頭初始化失敗 - {e}")
    
    def _start_frame_timer(self):
        """啟動影像更新定時器"""
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(16)  # 60 FPS
    
    def update_frame(self):
        """更新影像顯示"""
        # 檢查接收器是否已初始化
        if self.receiver is None:
            return
            
        # 從 FrameReceiver 獲取最新幀
        frame = self.receiver.get_latest_frame()
        if frame is None:
            return
        
        self.current_frame = frame
        self.frame_count += 1
        
        # 使用 ImageProcessingService 處理影像顯示
        label_width = self.image_label.width()
        label_height = self.image_label.height()
        
        display_frame, scale, offset_x, offset_y = self.image_service.prepare_frame_for_display(
            frame, label_width, label_height
        )
        
        if display_frame is None:
            return
        
        # 儲存顯示參數供 ROI 計算使用
        self.display_scale = scale
        self.display_offset_x = offset_x
        self.display_offset_y = offset_y
        
        # 繪製 ROI 覆蓋層
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
        
        # 轉換為 Qt 格式並顯示
        pixmap = self.image_service.convert_frame_to_qt_format(display_frame)
        self.image_label.setPixmap(pixmap)
        
        # 更新狀態（每100幀一次）
        if self.frame_count % 100 == 0:
            try:
                perf_summary = self.receiver.get_performance_summary()
                gpu_info = self.receiver.get_gpu_info()
                self.status_label.setText(f"系統狀態: {perf_summary} | GPU: {gpu_info['available']}")
                
                # 更新系統資源監控
                resource_summary = self.system_monitor.get_resource_summary()
                self.resource_label.setText(f"系統資源: {resource_summary}")
                
            except Exception as e:
                logger.warning(f"[OCR] Error updating status: {e}")
    
    def _get_roi_display_coords(self):
        """獲取 ROI 在顯示座標系中的座標"""
        if not (self.roi_start and self.roi_end):
            return None
        
        x1 = min(self.roi_start.x(), self.roi_end.x())
        y1 = min(self.roi_start.y(), self.roi_end.y())
        x2 = max(self.roi_start.x(), self.roi_end.x())
        y2 = max(self.roi_start.y(), self.roi_end.y())
        
        return (x1, y1, x2, y2)
    
    def eventFilter(self, obj, event):
        """處理 ROI 選擇的鼠標事件"""
        if obj == self.image_label and self.current_frame is not None and self.receiver is not None:
            if event.type() == event.MouseButtonPress:
                if event.button() == Qt.LeftButton:
                    mouse_pos = self._convert_mouse_coords(event.pos())
                    if mouse_pos:
                        self.roi_start = mouse_pos
                        self.roi_selecting = True
                        self.roi_selected = False
                        logger.info(f"[ROI] Start selection at: {mouse_pos.x()}, {mouse_pos.y()}")
                    return True
                    
            elif event.type() == event.MouseMove:
                if self.roi_selecting and self.roi_start:
                    mouse_pos = self._convert_mouse_coords(event.pos())
                    if mouse_pos:
                        self.roi_end = mouse_pos
                        self._update_roi_preview()
                    return True
                    
            elif event.type() == event.MouseButtonRelease:
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
        
        return super().eventFilter(obj, event)
    
    def _convert_mouse_coords(self, mouse_pos):
        """轉換鼠標座標到影像座標"""
        if self.current_frame is None:
            return None
            
        # 獲取當前影像尺寸作為參考
        img_h, img_w = self.current_frame.shape[:2]
        
        return self.image_service.convert_mouse_to_image_coords(
            mouse_pos, img_w, img_h,
            self.display_scale, self.display_offset_x, self.display_offset_y
        )
    
    def _update_roi_preview(self):
        """更新 ROI 預覽視窗"""
        if self.roi_start and self.roi_end and self.current_frame is not None:
            roi_coords = self._get_roi_display_coords()
            if roi_coords:
                roi_frame = self.image_service.extract_roi_from_frame(
                    self.current_frame, roi_coords, self.display_scale
                )
                self.roi_preview.update_preview(roi_frame)
    
    def toggle_roi_mode(self):
        """切換 ROI 選擇模式"""
        if self.roi_selected:
            self.clear_roi()
        else:
            self.roi_btn.setText('選擇辨識區域 (拖拽框選中...)')
            logger.info("[ROI] ROI selection mode enabled")
    
    def clear_roi(self):
        """清除 ROI 選擇"""
        self.roi_start = None
        self.roi_end = None
        self.roi_selecting = False
        self.roi_selected = False
        self.roi_btn.setText('選擇辨識區域 (拖拽框選)')
        self.roi_preview.hide()
        self.roi_preview.clear_preview()
        logger.info("[ROI] ROI selection cleared")
    
    def capture_and_ocr(self):
        """拍照並進行 OCR 辨識，並判斷許可/異常人員"""
        if self.current_frame is None:
            logger.warning("[OCR] No frame available for capture")
            return

        # 許可人員白名單，可改為讀取外部檔案
        whitelist = [
            {"id_number": "A123456789", "name": "王小明"},
            {"id_number": "B987654321", "name": "李小華"},
        ]

        def is_permitted(ocr_result, whitelist):
            for person in whitelist:
                if (ocr_result.get("id_number") == person["id_number"] and
                    ocr_result.get("name") == person["name"]):
                    return True
            return False

        try:
            # 決定處理區域
            if self.roi_selected and self.roi_start and self.roi_end:
                roi_coords = self._get_roi_display_coords()
                processed_frame = self.image_service.extract_roi_from_frame(
                    self.current_frame, roi_coords, self.display_scale
                )
                logger.info("[OCR] Using ROI region for OCR")
            else:
                processed_frame = self.current_frame
                logger.info("[OCR] Using full frame for OCR")

            if processed_frame is None:
                self.result_text.setText("錯誤：無法獲取有效的影像區域")
                return

            # 品質增強
            processed_frame = self.image_service.enhance_frame_quality(processed_frame)

            # 儲存暫存圖片
            import cv2
            img_path = 'temp_capture.jpg'
            cv2.imwrite(img_path, processed_frame)

            with open(img_path, 'rb') as f:
                image_bytes = f.read()

            logger.info(f"[OCR] Processing image with size: {processed_frame.shape}")

            # 呼叫 Gemini OCR
            img_part = {
                "mime_type": "image/jpeg",
                "data": image_bytes
            }

            resp = model.generate_content(
                [img_part, "這是一張台灣身份證照片，請回傳身分證號(id_number)與姓名(name)。"],
                generation_config=genai.GenerationConfig(
                    response_mime_type="application/json",
                    response_schema=TaiwanIDCard
                )
            )

            # 處理回應
            ocr_result = None
            result_text = ""
            if resp.candidates and resp.candidates[0].content and resp.candidates[0].content.parts:
                part = resp.candidates[0].content.parts[0]
                if part.function_call:
                    parsed_args = dict(part.function_call.args)
                    ocr_result = parsed_args
                    result_text = json.dumps(parsed_args, ensure_ascii=False, indent=2)
                elif resp.text:
                    try:
                        parsed_args = json.loads(resp.text)
                        ocr_result = parsed_args
                        result_text = resp.text
                    except json.JSONDecodeError:
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

            # 許可/異常判斷
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
            # 清理暫存檔案
            try:
                os.remove(img_path)
            except:
                pass
    
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

# ============================
# ========== Step 5 =============
# ====== 主程式入口點 ============
# ============================

def main():
    """主程式入口"""
    app = QApplication(sys.argv)
    try:
        window = OCRCameraWidget()
        window.show()
        sys.exit(app.exec_())
    except Exception as e:
        logger.error(f"[Main] Application error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
