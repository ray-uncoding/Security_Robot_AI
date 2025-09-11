"""影像處理服務"""
import cv2
import numpy as np
import logging
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QPoint
from ..utils.gpu_processor import GPUProcessor

logger = logging.getLogger(__name__)

class ImageProcessingService:
    """影像處理服務類，負責所有影像相關的操作"""
    
    def __init__(self):
        """初始化影像處理服務"""
        self.gpu_processor = GPUProcessor()
        logger.info(f"[ImageService] GPU acceleration: {self.gpu_processor.is_gpu_available()}")
    
    def prepare_frame_for_display(self, frame: np.ndarray, display_width: int, display_height: int) -> tuple:
        """
        準備影像用於顯示（保持長寬比）
        
        Args:
            frame: 原始影像
            display_width: 顯示區域寬度
            display_height: 顯示區域高度
            
        Returns:
            (display_frame, scale_factor, offset_x, offset_y)
        """
        if frame is None:
            return None, 0, 0, 0
        
        # 使用 GPU 處理器進行等比縮放
        display_frame, scale, offset_x, offset_y = self.gpu_processor.apply_scaling_with_aspect_ratio(
            frame, display_width, display_height
        )
        
        return display_frame, scale, offset_x, offset_y
    
    def convert_frame_to_qt_format(self, frame: np.ndarray) -> QPixmap:
        """
        將 OpenCV 影像轉換為 Qt 格式
        
        Args:
            frame: OpenCV 影像（BGR格式）
            
        Returns:
            Qt QPixmap 物件
        """
        if frame is None:
            return QPixmap()
        
        try:
            # 轉換色彩空間：BGR -> RGB
            rgb_frame = self.gpu_processor.convert_color_space(frame, cv2.COLOR_BGR2RGB)
            
            h, w, ch = rgb_frame.shape
            bytes_per_line = ch * w
            
            # 創建 QImage
            qt_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            
            # 轉換為 QPixmap
            pixmap = QPixmap.fromImage(qt_image)
            return pixmap
            
        except Exception as e:
            logger.error(f"[ImageService] Error converting frame to Qt format: {e}")
            return QPixmap()
    
    def extract_roi_from_frame(self, frame: np.ndarray, roi_coords: tuple, 
                              display_scale: float = 1.0) -> np.ndarray:
        """
        從影像中提取 ROI 區域
        
        Args:
            frame: 原始影像
            roi_coords: ROI 座標 (x1, y1, x2, y2)
            display_scale: 顯示縮放比例
            
        Returns:
            ROI 影像
        """
        if frame is None or not roi_coords:
            return None
        
        x1, y1, x2, y2 = roi_coords
        
        # 將顯示座標轉換為原始影像座標
        orig_x1 = int(x1 / display_scale)
        orig_y1 = int(y1 / display_scale)
        orig_x2 = int(x2 / display_scale)
        orig_y2 = int(y2 / display_scale)
        
        # 確保座標在影像範圍內
        h, w = frame.shape[:2]
        orig_x1 = max(0, min(orig_x1, w))
        orig_y1 = max(0, min(orig_y1, h))
        orig_x2 = max(0, min(orig_x2, w))
        orig_y2 = max(0, min(orig_y2, h))
        
        # 檢查 ROI 是否有效
        roi_width = orig_x2 - orig_x1
        roi_height = orig_y2 - orig_y1
        
        if roi_width <= 0 or roi_height <= 0:
            logger.warning(f"[ImageService] Invalid ROI size: {roi_width}x{roi_height}")
            return None
        
        # 使用 GPU 加速裁剪
        roi_frame = self.gpu_processor.crop_frame(frame, orig_x1, orig_y1, roi_width, roi_height)
        
        logger.info(f"[ImageService] Extracted ROI: {roi_width}x{roi_height} from {w}x{h}")
        return roi_frame
    
    def draw_roi_overlay(self, frame: np.ndarray, roi_coords: tuple, 
                        color: tuple = (0, 255, 0), thickness: int = 3, 
                        label: str = "ROI") -> np.ndarray:
        """
        在影像上繪製 ROI 覆蓋層
        
        Args:
            frame: 輸入影像
            roi_coords: ROI 座標 (x1, y1, x2, y2)
            color: 框線顏色 (B, G, R)
            thickness: 框線粗細
            label: 標籤文字
            
        Returns:
            繪製了 ROI 框的影像
        """
        if frame is None or not roi_coords:
            return frame
        
        # 複製影像以避免修改原始影像
        overlay_frame = frame.copy()
        
        x1, y1, x2, y2 = roi_coords
        
        # 繪製矩形框
        cv2.rectangle(overlay_frame, (x1, y1), (x2, y2), color, thickness)
        
        # 繪製標籤
        if label:
            label_y = max(y1 - 10, 15)
            cv2.putText(overlay_frame, label, (x1, label_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        return overlay_frame
    
    def convert_mouse_to_image_coords(self, mouse_pos: QPoint, display_width: int, 
                                    display_height: int, scale_factor: float, 
                                    offset_x: int, offset_y: int) -> QPoint:
        """
        將鼠標座標轉換為影像座標
        
        Args:
            mouse_pos: 鼠標位置
            display_width: 顯示區域寬度
            display_height: 顯示區域高度
            scale_factor: 縮放比例
            offset_x, offset_y: 偏移量
            
        Returns:
            轉換後的影像座標
        """
        # 減去偏移量，獲得在縮放影像中的座標
        img_x = mouse_pos.x() - offset_x
        img_y = mouse_pos.y() - offset_y
        
        # 計算縮放後的影像尺寸
        scaled_width = int(display_width * scale_factor)
        scaled_height = int(display_height * scale_factor)
        
        # 檢查是否在有效範圍內
        if 0 <= img_x <= scaled_width and 0 <= img_y <= scaled_height:
            return QPoint(img_x, img_y)
        else:
            logger.debug(f"[ImageService] Mouse outside image bounds: ({img_x}, {img_y}) vs ({scaled_width}, {scaled_height})")
            return None
    
    def enhance_frame_quality(self, frame: np.ndarray) -> np.ndarray:
        """
        增強影像品質
        
        Args:
            frame: 輸入影像
            
        Returns:
            增強後的影像
        """
        return self.gpu_processor.enhance_frame_quality(frame)
    
    def get_processing_info(self) -> dict:
        """
        獲取影像處理資訊
        
        Returns:
            處理器資訊字典
        """
        return {
            "gpu_info": self.gpu_processor.get_gpu_info(),
            "gpu_available": self.gpu_processor.is_gpu_available()
        }
