"""性能監控服務"""
import time
import threading
import logging
from typing import Dict, Optional
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)

@dataclass
class PerformanceMetrics:
    """性能指標資料結構"""
    fps: float = 0.0
    frame_count: int = 0
    processing_time: float = 0.0
    gpu_utilization: float = 0.0
    memory_usage: float = 0.0
    latency: float = 0.0
    last_update: float = field(default_factory=time.time)

class PerformanceMonitor:
    """性能監控器"""
    
    def __init__(self, update_interval: float = 2.0):
        """
        初始化性能監控器
        
        Args:
            update_interval: 更新間隔（秒）
        """
        self.update_interval = update_interval
        self.metrics = PerformanceMetrics()
        self._frame_times = []
        self._processing_times = []
        self._start_time = time.time()
        self._last_frame_time = time.time()
        self._monitoring = False
        self._monitor_thread = None
        self._lock = threading.Lock()
        
    def start_monitoring(self):
        """開始性能監控"""
        if self._monitoring:
            return
            
        self._monitoring = True
        self._monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._monitor_thread.start()
        logger.info("[Performance] Monitoring started")
    
    def stop_monitoring(self):
        """停止性能監控"""
        self._monitoring = False
        if self._monitor_thread:
            self._monitor_thread.join(timeout=1.0)
        logger.info("[Performance] Monitoring stopped")
    
    def record_frame(self):
        """記錄一個新幀"""
        current_time = time.time()
        
        with self._lock:
            # 計算幀間隔
            frame_interval = current_time - self._last_frame_time
            self._frame_times.append(frame_interval)
            
            # 保持最近100個樣本
            if len(self._frame_times) > 100:
                self._frame_times.pop(0)
            
            self._last_frame_time = current_time
            self.metrics.frame_count += 1
    
    def record_processing_time(self, processing_time: float):
        """記錄處理時間"""
        with self._lock:
            self._processing_times.append(processing_time)
            
            # 保持最近50個樣本
            if len(self._processing_times) > 50:
                self._processing_times.pop(0)
    
    def _monitor_loop(self):
        """監控循環"""
        while self._monitoring:
            self._update_metrics()
            time.sleep(self.update_interval)
    
    def _update_metrics(self):
        """更新性能指標"""
        current_time = time.time()
        
        with self._lock:
            # 計算 FPS
            if len(self._frame_times) > 1:
                avg_interval = sum(self._frame_times) / len(self._frame_times)
                self.metrics.fps = 1.0 / avg_interval if avg_interval > 0 else 0.0
            
            # 計算平均處理時間
            if self._processing_times:
                self.metrics.processing_time = sum(self._processing_times) / len(self._processing_times)
            
            # 計算延遲（最近幀的時間間隔）
            if self._frame_times:
                self.metrics.latency = self._frame_times[-1] * 1000  # 轉換為毫秒
            
            self.metrics.last_update = current_time
        
        # 每次更新時記錄調試信息
        if self.metrics.frame_count % 50 == 0:  # 每50幀記錄一次
            logger.debug(f"[Performance] FPS: {self.metrics.fps:.1f}, "
                        f"Processing: {self.metrics.processing_time*1000:.1f}ms, "
                        f"Latency: {self.metrics.latency:.1f}ms")
    
    def get_metrics(self) -> PerformanceMetrics:
        """獲取當前性能指標"""
        with self._lock:
            return PerformanceMetrics(
                fps=self.metrics.fps,
                frame_count=self.metrics.frame_count,
                processing_time=self.metrics.processing_time,
                gpu_utilization=self.metrics.gpu_utilization,
                memory_usage=self.metrics.memory_usage,
                latency=self.metrics.latency,
                last_update=self.metrics.last_update
            )
    
    def get_performance_summary(self) -> str:
        """獲取性能摘要字串"""
        metrics = self.get_metrics()
        return (f"FPS: {metrics.fps:.1f} | "
                f"處理時間: {metrics.processing_time*1000:.1f}ms | "
                f"延遲: {metrics.latency:.1f}ms | "
                f"總幀數: {metrics.frame_count}")
    
    def reset_metrics(self):
        """重設性能指標"""
        with self._lock:
            self.metrics = PerformanceMetrics()
            self._frame_times.clear()
            self._processing_times.clear()
            self._start_time = time.time()
            self._last_frame_time = time.time()
        
        logger.info("[Performance] Metrics reset")

class FrameRateController:
    """幀率控制器"""
    
    def __init__(self, target_fps: float = 60.0):
        """
        初始化幀率控制器
        
        Args:
            target_fps: 目標幀率
        """
        self.target_fps = target_fps
        self.target_interval = 1.0 / target_fps
        self.last_frame_time = time.time()
    
    def should_process_frame(self) -> bool:
        """檢查是否應該處理當前幀"""
        current_time = time.time()
        elapsed = current_time - self.last_frame_time
        
        if elapsed >= self.target_interval:
            self.last_frame_time = current_time
            return True
        return False
    
    def wait_for_next_frame(self):
        """等待到下一幀的時間"""
        current_time = time.time()
        elapsed = current_time - self.last_frame_time
        sleep_time = self.target_interval - elapsed
        
        if sleep_time > 0:
            time.sleep(sleep_time)
        
        self.last_frame_time = time.time()
    
    def set_target_fps(self, fps: float):
        """設定目標幀率"""
        self.target_fps = fps
        self.target_interval = 1.0 / fps
        logger.info(f"[FrameRate] Target FPS set to {fps}")

# 全域性能監控器實例
_global_monitor: Optional[PerformanceMonitor] = None

def get_global_performance_monitor() -> PerformanceMonitor:
    """獲取全域性能監控器"""
    global _global_monitor
    if _global_monitor is None:
        _global_monitor = PerformanceMonitor()
    return _global_monitor
