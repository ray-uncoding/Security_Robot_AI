# --- Qt5 + OpenCV + Gemini OCR ---
import sys
import os
import cv2
import json
import time
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QTextEdit
from PyQt5.QtCore import QTimer, Qt, QPoint
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtGui import QImage, QPixmap
from pydantic import BaseModel
import google.generativeai as genai

# --- GPU 加速設定 ---
# 檢查並啟用 CUDA 支援
if cv2.cuda.getCudaEnabledDeviceCount() > 0:
    USE_GPU = True
    print("[GPU] CUDA acceleration enabled!")
else:
    USE_GPU = False
    print("[GPU] CUDA not available, using CPU processing")

# 把專案根加入搜尋路徑（使 Insta_OpenCV 可被 import）
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from Insta_OpenCV.controller.insta_worker import InstaWorker
from Insta_OpenCV.utils.frame_receiver import FrameReceiver


# 定義台灣身份證 Schema
class TaiwanIDCard(BaseModel):
    id_number: str
    name: str

# Gemini client 初始化，需傳入 api_key
genai.configure(api_key=os.environ.get("GEMINI_API_KEY", "YOUR_API_KEY_HERE"))
model = genai.GenerativeModel('gemini-1.5-pro')

def crop_center_gpu(frame, crop_percentage=0.3):
    """GPU 加速的圖像裁剪函數"""
    try:
        # 上傳到 GPU
        gpu_frame = cv2.cuda_GpuMat()
        gpu_frame.upload(frame)
        
        # 修正：使用正確的屬性名
        h, w = gpu_frame.size()  # 返回 (width, height)
        w, h = h, w  # 交換順序以匹配 OpenCV 的 (height, width) 慣例
        
        start_y = int(h * (1 - crop_percentage) / 2)
        end_y = int(h * (1 + crop_percentage) / 2)
        start_x = int(w * (1 - crop_percentage) / 2)
        end_x = int(w * (1 + crop_percentage) / 2)
        
        # GPU 裁剪 (創建 ROI)
        gpu_cropped = cv2.cuda_GpuMat(gpu_frame, (start_x, start_y, end_x-start_x, end_y-start_y))
        
        # 下載回 CPU
        result = gpu_cropped.download()
        if len(result.shape) == 3:  # 確保是3通道圖像
            return result
        else:
            raise Exception("Invalid channel count after GPU crop")
        
    except Exception as e:
        print(f"[GPU] Error in GPU crop, using CPU: {e}")
        # 回退到 CPU 處理
        h, w = frame.shape[:2]
        start_y = int(h * (1 - crop_percentage) / 2)
        end_y = int(h * (1 + crop_percentage) / 2)
        start_x = int(w * (1 - crop_percentage) / 2)
        end_x = int(w * (1 + crop_percentage) / 2)
        return frame[start_y:end_y, start_x:end_x]

class ROIPreviewWindow(QWidget):
    """ROI區域高畫質預覽視窗"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle('ROI 高畫質預覽')
        self.setFixedSize(600, 400)
        
        # 移動到主視窗右側
        self.move(850, 100)
        
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
            # 計算適合視窗的縮放尺寸
            preview_w = self.preview_label.width() - 10
            preview_h = self.preview_label.height() - 10
            
            h, w = roi_frame.shape[:2]
            scale = min(preview_w / w, preview_h / h)
            new_w = int(w * scale)
            new_h = int(h * scale)
            
            # 縮放ROI影像
            resized_roi = cv2.resize(roi_frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
            
            # 轉換為Qt格式
            rgb_image = cv2.cvtColor(resized_roi, cv2.COLOR_BGR2RGB)
            qt_image = QImage(rgb_image.data, new_w, new_h, new_w * 3, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            
            self.preview_label.setPixmap(pixmap)
            self.info_label.setText(f"ROI解析度: {w}x{h} | 預覽: {new_w}x{new_h}")
        else:
            self.preview_label.clear()
            self.preview_label.setText("選擇ROI區域後\n將顯示高畫質預覽")
            self.info_label.setText("尚未選擇區域")
    
    def clear_preview(self):
        """清除預覽"""
        self.preview_label.clear()
        self.preview_label.setText("選擇ROI區域後\n將顯示高畫質預覽")
        self.info_label.setText("尚未選擇區域")

class CameraWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Gemini OCR 身份證辨識 (Insta360 Preview + ROI選擇)')
        
        # Set window size
        self.setFixedSize(800, 900)  # Width x Height
        
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(780, 440)  # Set minimum size for video display (16:9 aspect ratio)
        self.image_label.setStyleSheet("border: 2px solid gray;")  # Add border to see the label clearly
        self.image_label.setMouseTracking(True)
        
        # ROI 選擇相關變數
        self.roi_start = None
        self.roi_end = None
        self.roi_selecting = False
        self.roi_selected = False
        self.roi_rect = None  # 儲存選中的區域
        
        # 安裝事件過濾器來處理鼠標事件
        self.image_label.installEventFilter(self)
        
        self.capture_btn = QPushButton('拍照並辨識')
        self.capture_btn.clicked.connect(self.capture_and_ocr)
        self.capture_btn.setMinimumHeight(40)  # Make button a bit taller
        
        # 新增ROI相關按鈕
        self.roi_btn = QPushButton('選擇辨識區域 (拖拽框選)')
        self.roi_btn.clicked.connect(self.toggle_roi_mode)
        self.roi_btn.setMinimumHeight(40)
        
        self.clear_roi_btn = QPushButton('清除選擇區域')
        self.clear_roi_btn.clicked.connect(self.clear_roi)
        self.clear_roi_btn.setMinimumHeight(40)
        
        self.result_text = QTextEdit()
        self.result_text.setReadOnly(True)
        self.result_text.setMinimumHeight(200)  # Set minimum height for result text
        
        # 按鈕佈局
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.roi_btn)
        button_layout.addWidget(self.clear_roi_btn)
        button_layout.addWidget(self.capture_btn)
        
        layout = QVBoxLayout()
        layout.addWidget(self.image_label, 1)  # Give most space to image
        layout.addLayout(button_layout, 0)  # Fixed size for buttons
        layout.addWidget(self.result_text, 0)  # Fixed size for text
        self.setLayout(layout)

        # --- Insta360 Camera Setup ---
        print("[INFO] Init InstaWorker ...")
        # Pass the correct IP address for the Insta360 camera
        self.worker = InstaWorker(ip_address="192.168.1.188")
        ready_event = self.worker.start_preview_all()  # 啟用 preview 模式
        print("[INFO] Waiting for worker to be ready...")
        ready_event.wait()
        print("[INFO] Worker ready! 等待推流穩定...")
        time.sleep(5)  # 等待推流穩定

        # --- Correctly initialize FrameReceiver with the RTMP URL ---
        # Try to pull stream directly from camera's RTMP server instead of expecting camera to push to us
        camera_rtmp_url = "rtmp://192.168.1.188:1935/live/preview"
        self.receiver = FrameReceiver(stream_url=camera_rtmp_url) # Pass URL as keyword argument
        self.receiver.start()
        # --- End Insta360 Setup ---

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(16)  # 提升到16ms (約60 FPS) 以獲得更流暢的體驗
        self.current_frame = None
        
        # 性能優化變數
        self._frame_count = 0
        self._skip_frames = 0  # 幀跳過計數器
        
        # 創建ROI預覽視窗
        self.roi_preview = ROIPreviewWindow()
        self.roi_preview.hide()  # 初始隱藏

    def eventFilter(self, obj, event):
        """處理圖像標籤上的鼠標事件來實現ROI選擇"""
        if obj == self.image_label and hasattr(self, 'current_frame') and self.current_frame is not None:
            # 獲取當前影像和標籤的尺寸信息
            if not hasattr(self, 'display_scale'):
                return super().eventFilter(obj, event)
                
            if event.type() == event.MouseButtonPress:
                if event.button() == Qt.LeftButton:
                    # 轉換鼠標座標到影像座標
                    mouse_pos = self.convert_mouse_to_image_coords(event.pos())
                    if mouse_pos:
                        self.roi_start = mouse_pos
                        self.roi_selecting = True
                        self.roi_selected = False
                        print(f"[ROI] Start selection at: {mouse_pos}")
                    return True
                    
            elif event.type() == event.MouseMove:
                if self.roi_selecting and self.roi_start:
                    # 轉換鼠標座標到影像座標
                    mouse_pos = self.convert_mouse_to_image_coords(event.pos())
                    if mouse_pos:
                        self.roi_end = mouse_pos
                        # 即時更新預覽
                        self.update_roi_preview()
                    return True
                    
            elif event.type() == event.MouseButtonRelease:
                if event.button() == Qt.LeftButton and self.roi_selecting:
                    # 轉換鼠標座標到影像座標
                    mouse_pos = self.convert_mouse_to_image_coords(event.pos())
                    if mouse_pos:
                        self.roi_end = mouse_pos
                        self.roi_selecting = False
                        self.roi_selected = True
                        # 顯示並更新預覽視窗
                        self.roi_preview.show()
                        self.update_roi_preview()
                        print(f"[ROI] Selection completed: {self.roi_start} to {self.roi_end}")
                    return True
                    
        return super().eventFilter(obj, event)
    
    def convert_mouse_to_image_coords(self, mouse_pos):
        """將鼠標座標轉換為顯示影像座標"""
        if not hasattr(self, 'display_scale') or not hasattr(self, 'current_frame'):
            return None
            
        # 獲取標籤和影像尺寸
        label_w = self.image_label.width()
        label_h = self.image_label.height()
        
        img_h, img_w = self.current_frame.shape[:2]
        scale = min(label_w / img_w, label_h / img_h)
        new_w = int(img_w * scale)
        new_h = int(img_h * scale)
        
        # 計算影像在標籤中的偏移
        offset_x = (label_w - new_w) // 2
        offset_y = (label_h - new_h) // 2
        
        # 轉換鼠標座標到影像座標
        img_x = mouse_pos.x() - offset_x
        img_y = mouse_pos.y() - offset_y
        
        # 檢查是否在影像範圍內
        if 0 <= img_x <= new_w and 0 <= img_y <= new_h:
            # 創建 QPoint 物件返回
            from PyQt5.QtCore import QPoint
            return QPoint(img_x, img_y)
        else:
            print(f"[ROI] Mouse outside image bounds: ({img_x}, {img_y})")
            return None

    def toggle_roi_mode(self):
        """切換ROI選擇模式"""
        if self.roi_selected:
            self.clear_roi()
        else:
            self.roi_btn.setText('選擇辨識區域 (拖拽框選中...)')
            print("[ROI] ROI selection mode enabled. Click and drag to select region.")

    def clear_roi(self):
        """清除ROI選擇"""
        self.roi_start = None
        self.roi_end = None
        self.roi_selecting = False
        self.roi_selected = False
        self.roi_rect = None
        self.roi_btn.setText('選擇辨識區域 (拖拽框選)')
        # 隱藏並清除預覽視窗
        self.roi_preview.hide()
        self.roi_preview.clear_preview()
        print("[ROI] ROI selection cleared.")
        
    def update_roi_preview(self):
        """更新ROI預覽視窗"""
        if self.roi_start and self.roi_end and self.current_frame is not None:
            roi_frame = self.extract_roi_from_original()
            self.roi_preview.update_preview(roi_frame)

    def update_frame(self):
        # 移除幀跳過機制以減少延遲
        frame = self.receiver.get_latest_frame() # Get frame from Insta360
        if frame is not None:
            # 減少調試輸出以提高性能
            self._frame_count += 1
                
            # 只每50幀輸出一次調試信息 (減少輸出頻率)
            debug_output = (self._frame_count % 50 == 0)
            if debug_output:
                print(f"[update_frame] Frame #{self._frame_count} - Original size: {frame.shape}")
            
            # 使用等比縮放代替裁切，保留完整影像
            self.current_frame = frame  # 不進行裁切，保持原始影像
            
            # --- GPU 加速的等比縮放處理 ---
            label_w = self.image_label.width()
            label_h = self.image_label.height()
            
            try:
                # 計算等比縮放尺寸，保持長寬比
                img_h, img_w = self.current_frame.shape[:2]
                scale = min(label_w / img_w, label_h / img_h)
                new_w = int(img_w * scale)
                new_h = int(img_h * scale)
                
                # GPU 等比縮放
                gpu_frame = cv2.cuda_GpuMat()
                gpu_frame.upload(self.current_frame)
                
                gpu_resized = cv2.cuda.resize(gpu_frame, (new_w, new_h), 
                                            interpolation=cv2.INTER_LINEAR)
                
                # 下載並用 CPU 做色彩轉換
                display_frame = gpu_resized.download()
                
                # 如果有ROI選擇，繪製選擇框
                if self.roi_selected and self.roi_start and self.roi_end:
                    # 在縮放後的影像上繪製ROI框
                    x1 = min(self.roi_start.x(), self.roi_end.x())
                    y1 = min(self.roi_start.y(), self.roi_end.y())
                    x2 = max(self.roi_start.x(), self.roi_end.x())
                    y2 = max(self.roi_start.y(), self.roi_end.y())
                    
                    # 確保座標在影像範圍內
                    x1 = max(0, min(x1, new_w))
                    y1 = max(0, min(y1, new_h))
                    x2 = max(0, min(x2, new_w))
                    y2 = max(0, min(y2, new_h))
                    
                    # 繪製ROI框
                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    cv2.putText(display_frame, "OCR Region", (x1, max(y1-10, 15)), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                elif self.roi_selecting and self.roi_start and self.roi_end:
                    # 正在選擇時繪製半透明框
                    x1 = min(self.roi_start.x(), self.roi_end.x())
                    y1 = min(self.roi_start.y(), self.roi_end.y())
                    x2 = max(self.roi_start.x(), self.roi_end.x())
                    y2 = max(self.roi_start.y(), self.roi_end.y())
                    
                    # 確保座標在影像範圍內
                    x1 = max(0, min(x1, new_w))
                    y1 = max(0, min(y1, new_h))
                    x2 = max(0, min(x2, new_w))
                    y2 = max(0, min(y2, new_h))
                    
                    # 繪製選擇中的框
                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), (255, 255, 0), 2)
                
                rgb_image = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
                
                # 儲存縮放比例供ROI計算使用
                self.display_scale = scale
                self.display_offset_x = (label_w - new_w) // 2
                self.display_offset_y = (label_h - new_h) // 2
                
                if debug_output:
                    print(f"[GPU] Aspect ratio: {img_w}x{img_h} -> {new_w}x{new_h}")
                        
            except Exception as e:
                if debug_output:
                    print(f"[GPU] Using CPU fallback: {e}")
                # GPU 失敗時使用 CPU 等比縮放
                img_h, img_w = self.current_frame.shape[:2]
                scale = min(label_w / img_w, label_h / img_h)
                new_w = int(img_w * scale)
                new_h = int(img_h * scale)
                display_frame = cv2.resize(self.current_frame, (new_w, new_h), 
                                         interpolation=cv2.INTER_LINEAR)
                rgb_image = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
                self.display_scale = scale
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.image_label.setPixmap(pixmap)
            
            if debug_output:
                print(f"[update_frame] Display updated: {w}x{h}")
        else:
            print("[update_frame] No frame available")

    def capture_and_ocr(self):
        if self.current_frame is not None:
            # 決定要處理的影像區域
            if self.roi_selected and self.roi_start and self.roi_end:
                # 使用選中的ROI區域
                processed_frame = self.extract_roi_from_original()
                print(f"[OCR] Using ROI region for high-quality OCR")
            else:
                # 使用完整影像
                processed_frame = self.current_frame
                print(f"[OCR] Using full frame for OCR")
            
            if processed_frame is None:
                print("[OCR] Error: No valid frame to process")
                return
                
            # 儲存暫存圖片
            img_path = 'temp_capture.jpg'
            cv2.imwrite(img_path, processed_frame)
            with open(img_path, 'rb') as f:
                image_bytes = f.read()
            print(f"[OCR] Processing image with size: {processed_frame.shape}")
            
            # 呼叫 Gemini OCR
            try:
                # Create the image part
                img_part = {
                    "mime_type": "image/jpeg",
                    "data": image_bytes
                }
                # Make the API call
                resp = model.generate_content(
                    [img_part, "這是一張台灣身份證照片，請回傳身分證號(id_number)與姓名(name)。"],
                    generation_config=genai.GenerationConfig(
                        response_mime_type="application/json",
                        response_schema=TaiwanIDCard
                    )
                )

                # The model should return a function call, but if not, try to parse the text.
                if resp.candidates and resp.candidates[0].content and resp.candidates[0].content.parts:
                    part = resp.candidates[0].content.parts[0]
                    if part.function_call:
                        parsed_args = dict(part.function_call.args)
                        self.result_text.setText(json.dumps(parsed_args, ensure_ascii=False, indent=2))
                        self.result_text.append(f"\n---\nPython物件：\n身分證號: {parsed_args.get('id_number', 'N/A')}\n姓名: {parsed_args.get('name', 'N/A')}")
                    elif resp.text:
                        try:
                            # The model might just return a json string in text
                            parsed_args = json.loads(resp.text)
                            self.result_text.setText(resp.text)
                            self.result_text.append(f"\n---\nPython物件：\n身分證號: {parsed_args.get('id_number', 'N/A')}\n姓名: {parsed_args.get('name', 'N/A')}")
                        except json.JSONDecodeError:
                             self.result_text.setText(f"無法解析OCR回應。\n\n原始回應:\n{resp.text}")
                    else:
                        self.result_text.setText("無法解析OCR回應，且沒有收到文字回應。")
                else:
                    # Handle cases where the response is not as expected
                    error_message = "無法解析OCR回應。請再試一次。"
                    if resp.prompt_feedback and resp.prompt_feedback.block_reason:
                        error_message += f"\n原因: {resp.prompt_feedback.block_reason.name}"
                    
                    if resp.text:
                        error_message += f"\n\n原始回應:\n{resp.text}"

                    self.result_text.setText(error_message)
                    
            except Exception as e:
                self.result_text.setText(f"OCR 發生錯誤：{e}")
            finally:
                os.remove(img_path)

    def extract_roi_from_original(self):
        """從原始高解析度影像中提取ROI區域"""
        if not (self.roi_selected and self.roi_start and self.roi_end and hasattr(self, 'display_scale')):
            return None
            
        # 獲取顯示影像的座標
        x1 = min(self.roi_start.x(), self.roi_end.x())
        y1 = min(self.roi_start.y(), self.roi_end.y())
        x2 = max(self.roi_start.x(), self.roi_end.x())
        y2 = max(self.roi_start.y(), self.roi_end.y())
        
        # 轉換回原始影像座標
        orig_x1 = int(x1 / self.display_scale)
        orig_y1 = int(y1 / self.display_scale)
        orig_x2 = int(x2 / self.display_scale)
        orig_y2 = int(y2 / self.display_scale)
        
        # 確保座標在影像範圍內
        orig_h, orig_w = self.current_frame.shape[:2]
        orig_x1 = max(0, min(orig_x1, orig_w))
        orig_y1 = max(0, min(orig_y1, orig_h))
        orig_x2 = max(0, min(orig_x2, orig_w))
        orig_y2 = max(0, min(orig_y2, orig_h))
        
        # 檢查ROI是否有效
        if orig_x2 <= orig_x1 or orig_y2 <= orig_y1:
            print("[ROI] Invalid ROI region")
            return None
        
        # 提取ROI區域
        roi_frame = self.current_frame[orig_y1:orig_y2, orig_x1:orig_x2]
        
        print(f"[ROI] Display coords: ({x1},{y1}) to ({x2},{y2})")
        print(f"[ROI] Original coords: ({orig_x1},{orig_y1}) to ({orig_x2},{orig_y2})")
        print(f"[ROI] Extracted size: {roi_frame.shape}")
        
        return roi_frame

    def closeEvent(self, event):
        print('[INFO] Stopping camera and workers...')
        self.timer.stop()
        self.receiver.stop()
        self.worker.stop_all()
        # 關閉ROI預覽視窗
        if hasattr(self, 'roi_preview'):
            self.roi_preview.close()
        print('[INFO] All stopped.')
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = CameraWidget()
    window.resize(800, 600)
    window.show()
    sys.exit(app.exec_())