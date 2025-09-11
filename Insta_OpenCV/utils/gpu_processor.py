"""GPU 加速影像處理模組"""
import cv2
import numpy as np
import logging

logger = logging.getLogger(__name__)

class GPUProcessor:
    """GPU 加速影像處理器"""
    
    def __init__(self):
        """初始化 GPU 處理器"""
        self.gpu_available = self._check_gpu_availability()
        self.gpu_frame_cache = None
        self._last_resize_params = None
        
    def _check_gpu_availability(self) -> bool:
        """檢查 GPU 可用性"""
        try:
            gpu_count = cv2.cuda.getCudaEnabledDeviceCount()
            if gpu_count > 0:
                logger.info(f"[GPU] CUDA acceleration enabled! Devices: {gpu_count}")
                return True
            else:
                logger.warning("[GPU] CUDA not available, using CPU processing")
                return False
        except Exception as e:
            logger.error(f"[GPU] Error checking CUDA availability: {e}")
            return False
    
    def resize_frame(self, frame: np.ndarray, target_width: int, target_height: int, 
                    interpolation: int = cv2.INTER_LINEAR) -> np.ndarray:
        """
        GPU 加速的影像縮放
        
        Args:
            frame: 輸入影像
            target_width: 目標寬度
            target_height: 目標高度
            interpolation: 插值方法
            
        Returns:
            縮放後的影像
        """
        if not self.gpu_available:
            return cv2.resize(frame, (target_width, target_height), interpolation=interpolation)
        
        try:
            # 上傳到 GPU
            gpu_frame = cv2.cuda_GpuMat()
            gpu_frame.upload(frame)
            
            # GPU 縮放
            gpu_resized = cv2.cuda.resize(gpu_frame, (target_width, target_height), 
                                        interpolation=interpolation)
            
            # 下載回 CPU
            result = gpu_resized.download()
            return result
            
        except Exception as e:
            logger.warning(f"[GPU] Resize failed, using CPU fallback: {e}")
            return cv2.resize(frame, (target_width, target_height), interpolation=interpolation)
    
    def crop_frame(self, frame: np.ndarray, x: int, y: int, width: int, height: int) -> np.ndarray:
        """
        GPU 加速的影像裁剪
        
        Args:
            frame: 輸入影像
            x, y: 裁剪起始座標
            width, height: 裁剪尺寸
            
        Returns:
            裁剪後的影像
        """
        if not self.gpu_available:
            return frame[y:y+height, x:x+width]
        
        try:
            # 上傳到 GPU
            gpu_frame = cv2.cuda_GpuMat()
            gpu_frame.upload(frame)
            
            # GPU 裁剪 (創建 ROI)
            gpu_cropped = cv2.cuda_GpuMat(gpu_frame, (x, y, width, height))
            
            # 下載回 CPU
            result = gpu_cropped.download()
            return result
            
        except Exception as e:
            logger.warning(f"[GPU] Crop failed, using CPU fallback: {e}")
            return frame[y:y+height, x:x+width]
    
    def apply_scaling_with_aspect_ratio(self, frame: np.ndarray, max_width: int, max_height: int) -> tuple:
        """
        保持長寬比的縮放
        
        Args:
            frame: 輸入影像
            max_width: 最大寬度
            max_height: 最大高度
            
        Returns:
            (scaled_frame, scale_factor, offset_x, offset_y)
        """
        h, w = frame.shape[:2]
        
        # 計算縮放比例，保持長寬比
        scale = min(max_width / w, max_height / h)
        new_w = int(w * scale)
        new_h = int(h * scale)
        
        # 縮放影像
        scaled_frame = self.resize_frame(frame, new_w, new_h)
        
        # 計算置中偏移
        offset_x = (max_width - new_w) // 2
        offset_y = (max_height - new_h) // 2
        
        return scaled_frame, scale, offset_x, offset_y
    
    def convert_color_space(self, frame: np.ndarray, conversion_code: int) -> np.ndarray:
        """
        GPU 加速的色彩空間轉換
        
        Args:
            frame: 輸入影像
            conversion_code: OpenCV 色彩轉換代碼
            
        Returns:
            轉換後的影像
        """
        if not self.gpu_available:
            return cv2.cvtColor(frame, conversion_code)
        
        try:
            # 上傳到 GPU
            gpu_frame = cv2.cuda_GpuMat()
            gpu_frame.upload(frame)
            
            # GPU 色彩轉換
            gpu_converted = cv2.cuda.cvtColor(gpu_frame, conversion_code)
            
            # 下載回 CPU
            result = gpu_converted.download()
            return result
            
        except Exception as e:
            logger.warning(f"[GPU] Color conversion failed, using CPU fallback: {e}")
            return cv2.cvtColor(frame, conversion_code)
    
    def enhance_frame_quality(self, frame: np.ndarray) -> np.ndarray:
        """
        GPU 加速的影像品質增強
        
        Args:
            frame: 輸入影像
            
        Returns:
            增強後的影像
        """
        if not self.gpu_available:
            # CPU 版本的簡單增強
            return cv2.convertScaleAbs(frame, alpha=1.1, beta=10)
        
        try:
            # 上傳到 GPU
            gpu_frame = cv2.cuda_GpuMat()
            gpu_frame.upload(frame)
            
            # GPU 增強處理
            gpu_enhanced = cv2.cuda.convertScaleAbs(gpu_frame, alpha=1.1, beta=10)
            
            # 下載回 CPU
            result = gpu_enhanced.download()
            return result
            
        except Exception as e:
            logger.warning(f"[GPU] Enhancement failed, using CPU fallback: {e}")
            return cv2.convertScaleAbs(frame, alpha=1.1, beta=10)
    
    def is_gpu_available(self) -> bool:
        """檢查 GPU 是否可用"""
        return self.gpu_available
    
    def get_gpu_info(self) -> dict:
        """獲取 GPU 資訊"""
        if not self.gpu_available:
            return {"available": False, "device_count": 0}
        
        try:
            device_count = cv2.cuda.getCudaEnabledDeviceCount()
            return {
                "available": True,
                "device_count": device_count,
                "current_device": cv2.cuda.getDevice() if device_count > 0 else -1
            }
        except Exception as e:
            logger.error(f"[GPU] Error getting GPU info: {e}")
            return {"available": False, "error": str(e)}
