"""系統資源監控服務"""
import psutil
import time
import threading
import logging
from typing import Dict, Optional
from dataclasses import dataclass
import subprocess
import re

logger = logging.getLogger(__name__)

@dataclass
class SystemMetrics:
    """系統資源指標"""
    cpu_percent: float = 0.0
    memory_percent: float = 0.0
    memory_used_gb: float = 0.0
    memory_total_gb: float = 0.0
    gpu_percent: float = 0.0
    gpu_memory_used_mb: float = 0.0
    gpu_memory_total_mb: float = 0.0
    gpu_temp: float = 0.0
    cpu_temp: float = 0.0
    timestamp: float = 0.0

class SystemMonitor:
    """系統資源監控器"""
    
    def __init__(self, update_interval: float = 2.0):
        """
        初始化系統監控器
        
        Args:
            update_interval: 更新間隔（秒）
        """
        self.update_interval = update_interval
        self.metrics = SystemMetrics()
        self._monitoring = False
        self._monitor_thread = None
        self._lock = threading.Lock()
        
        # 檢查 Jetson 特定功能
        self.is_jetson = self._detect_jetson()
        logger.info(f"[SystemMonitor] Jetson platform detected: {self.is_jetson}")
        
    def _detect_jetson(self) -> bool:
        """檢測是否為 Jetson 平台"""
        try:
            result = subprocess.run(['jetson_release', '-v'], 
                                  capture_output=True, text=True, timeout=5)
            return result.returncode == 0
        except:
            return False
    
    def start_monitoring(self):
        """開始監控"""
        if self._monitoring:
            return
            
        self._monitoring = True
        self._monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._monitor_thread.start()
        logger.info("[SystemMonitor] Resource monitoring started")
    
    def stop_monitoring(self):
        """停止監控"""
        self._monitoring = False
        if self._monitor_thread:
            self._monitor_thread.join(timeout=1.0)
        logger.info("[SystemMonitor] Resource monitoring stopped")
    
    def _monitor_loop(self):
        """監控循環"""
        while self._monitoring:
            try:
                self._update_metrics()
                time.sleep(self.update_interval)
            except Exception as e:
                logger.error(f"[SystemMonitor] Error in monitor loop: {e}")
                time.sleep(self.update_interval)
    
    def _update_metrics(self):
        """更新系統指標"""
        current_time = time.time()
        
        with self._lock:
            # CPU 和記憶體指標
            self.metrics.cpu_percent = psutil.cpu_percent(interval=0.1)
            
            memory = psutil.virtual_memory()
            self.metrics.memory_percent = memory.percent
            self.metrics.memory_used_gb = memory.used / (1024**3)
            self.metrics.memory_total_gb = memory.total / (1024**3)
            
            # GPU 指標
            if self.is_jetson:
                self._update_jetson_gpu_metrics()
            else:
                self._update_generic_gpu_metrics()
            
            # 溫度指標
            self._update_temperature_metrics()
            
            self.metrics.timestamp = current_time
    
    def _update_jetson_gpu_metrics(self):
        """更新 Jetson GPU 指標"""
        try:
            # 嘗試使用 jtop 的 jetson_stats 模組
            try:
                from jtop import jtop
                with jtop() as jetson:
                    if jetson.ok():
                        # 獲取 GPU 統計
                        gpu_stats = jetson.gpu
                        if gpu_stats:
                            # 新的jtop格式: gpu_stats = {'gpu': {'status': {'load': 62.2}, ...}}
                            if 'gpu' in gpu_stats:
                                gpu_data = gpu_stats['gpu']
                                if 'status' in gpu_data and 'load' in gpu_data['status']:
                                    self.metrics.gpu_percent = float(gpu_data['status']['load'])
                            # 舊格式兼容性檢查
                            elif isinstance(gpu_stats, dict):
                                for gpu_name, gpu_data in gpu_stats.items():
                                    if isinstance(gpu_data, dict) and 'val' in gpu_data:
                                        self.metrics.gpu_percent = float(gpu_data['val'])
                                        break
                        
                        # 獲取溫度統計
                        temp_stats = jetson.temperature
                        if temp_stats and 'gpu' in temp_stats:
                            gpu_temp_data = temp_stats['gpu']
                            if isinstance(gpu_temp_data, dict) and 'temp' in gpu_temp_data:
                                self.metrics.gpu_temp = float(gpu_temp_data['temp'])
                            elif isinstance(gpu_temp_data, (int, float)):
                                self.metrics.gpu_temp = float(gpu_temp_data)
                        
                        # GPU記憶體資訊通常在Jetson上不直接可用，但可以嘗試獲取
                        memory_stats = jetson.memory
                        if memory_stats and 'EMC' in memory_stats:
                            # EMC是記憶體控制器，可以作為GPU記憶體的參考
                            emc_data = memory_stats['EMC']
                            if isinstance(emc_data, dict) and 'val' in emc_data:
                                # 這是一個近似值，因為Jetson使用統一記憶體架構
                                emc_usage = emc_data['val']
                                # 假設GPU使用整體記憶體的一部分
                                total_ram = memory_stats.get('RAM', {}).get('tot', 0)
                                if total_ram > 0:
                                    # 估算GPU記憶體使用（這是一個粗略估算）
                                    estimated_gpu_mem = (emc_usage / 100.0) * (total_ram / (1024 * 1024))  # 轉換為MB
                                    self.metrics.gpu_memory_used_mb = estimated_gpu_mem * 0.3  # 假設GPU使用30%的系統記憶體
                                    self.metrics.gpu_memory_total_mb = total_ram / (1024 * 1024) * 0.3
                        
                        return
                        
            except ImportError:
                logger.debug("[SystemMonitor] jtop module not available")
            except Exception as e:
                logger.debug(f"[SystemMonitor] jtop access error: {e}")
            
            # 嘗試從 Jetson 系統文件讀取
            try:
                # 讀取GPU頻率信息來估算使用率
                with open('/sys/devices/platform/bus@0/17000000.gpu/devfreq/17000000.gpu/cur_freq', 'r') as f:
                    cur_freq = int(f.read().strip())
                with open('/sys/devices/platform/bus@0/17000000.gpu/devfreq/17000000.gpu/max_freq', 'r') as f:
                    max_freq = int(f.read().strip())
                
                # 基於頻率計算近似使用率
                if max_freq > 0:
                    freq_ratio = cur_freq / max_freq
                    # 這只是一個粗略的估算，真實的GPU使用率需要其他方法
                    self.metrics.gpu_percent = min(freq_ratio * 100, 100.0)
                    
            except Exception as e:
                logger.debug(f"[SystemMonitor] Jetson sysfs GPU metrics error: {e}")
                
        except Exception as e:
            logger.debug(f"[SystemMonitor] Jetson GPU metrics error: {e}")
            # 回退到通用方法
            self._update_generic_gpu_metrics()
    
    def _update_generic_gpu_metrics(self):
        """更新通用 GPU 指標"""
        try:
            # 嘗試使用 nvidia-smi
            result = subprocess.run(['nvidia-smi', '--query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu', 
                                   '--format=csv,noheader,nounits'], 
                                  capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                values = result.stdout.strip().split(', ')
                if len(values) >= 4:
                    self.metrics.gpu_percent = float(values[0])
                    self.metrics.gpu_memory_used_mb = float(values[1])
                    self.metrics.gpu_memory_total_mb = float(values[2])
                    self.metrics.gpu_temp = float(values[3])
                    
        except Exception as e:
            logger.debug(f"[SystemMonitor] Generic GPU metrics error: {e}")
            # 設定預設值
            self.metrics.gpu_percent = 0.0
            self.metrics.gpu_memory_used_mb = 0.0
            self.metrics.gpu_memory_total_mb = 0.0
    
    def _update_temperature_metrics(self):
        """更新溫度指標"""
        try:
            if hasattr(psutil, "sensors_temperatures"):
                temps = psutil.sensors_temperatures()
                if temps:
                    # 嘗試獲取 CPU 溫度
                    for name, entries in temps.items():
                        if 'coretemp' in name.lower() or 'cpu' in name.lower():
                            if entries:
                                self.metrics.cpu_temp = entries[0].current
                                break
        except Exception as e:
            logger.debug(f"[SystemMonitor] Temperature metrics error: {e}")
    
    def get_metrics(self) -> SystemMetrics:
        """獲取當前系統指標"""
        with self._lock:
            return SystemMetrics(
                cpu_percent=self.metrics.cpu_percent,
                memory_percent=self.metrics.memory_percent,
                memory_used_gb=self.metrics.memory_used_gb,
                memory_total_gb=self.metrics.memory_total_gb,
                gpu_percent=self.metrics.gpu_percent,
                gpu_memory_used_mb=self.metrics.gpu_memory_used_mb,
                gpu_memory_total_mb=self.metrics.gpu_memory_total_mb,
                gpu_temp=self.metrics.gpu_temp,
                cpu_temp=self.metrics.cpu_temp,
                timestamp=self.metrics.timestamp
            )
    
    def get_resource_summary(self) -> str:
        """獲取資源使用摘要"""
        metrics = self.get_metrics()
        
        summary = (f"CPU: {metrics.cpu_percent:.1f}% | "
                  f"GPU: {metrics.gpu_percent:.1f}% | "
                  f"RAM: {metrics.memory_used_gb:.1f}G/{metrics.memory_total_gb:.1f}G "
                  f"({metrics.memory_percent:.1f}%)")
        
        if metrics.gpu_memory_total_mb > 0:
            gpu_mem_gb = metrics.gpu_memory_used_mb / 1024
            gpu_total_gb = metrics.gpu_memory_total_mb / 1024
            summary += f" | GPU RAM: {gpu_mem_gb:.1f}G/{gpu_total_gb:.1f}G"
        
        if metrics.cpu_temp > 0:
            summary += f" | CPU溫度: {metrics.cpu_temp:.1f}°C"
        
        if metrics.gpu_temp > 0:
            summary += f" | GPU溫度: {metrics.gpu_temp:.1f}°C"
        
        return summary

# 全域系統監控器實例
_global_system_monitor: Optional[SystemMonitor] = None

def get_global_system_monitor() -> SystemMonitor:
    """獲取全域系統監控器"""
    global _global_system_monitor
    if _global_system_monitor is None:
        _global_system_monitor = SystemMonitor()
    return _global_system_monitor
