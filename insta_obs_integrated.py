#!/usr/bin/env python3
"""
Insta360 + OBS 整合解決方案
整合 Insta Worker 和 OBS 串流捕獲功能

工作流程:
1. 啟動 Insta Worker (推流到 RTMP)
2. 使用 OpenCV 從 RTMP 串流捕獲視頻和音頻
3. 實時顯示和處理
"""

import os
import sys
import time
import cv2
import json
import logging
import threading
import numpy as np
from typing import Optional, Dict, Any

# 將專案根目錄加入 Python 路徑
project_root = os.path.dirname(os.path.abspath(__file__))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# 導入 Insta 相關模組
from Insta_OpenCV.controller.insta_worker import InstaWorker
from Insta_OpenCV.utils.frame_receiver import FrameReceiver
from Insta_OpenCV.services.image_processing import ImageProcessingService
from Insta_OpenCV.services.system_monitor import get_global_system_monitor

# 導入我們的 OBS 捕獲模組
from Insta_OpenCV.obs_stream_capture import OBSStreamCapture
from Insta_OpenCV.obs_audio import OBSAudioCapture

# 設置日誌
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class InstaOBSManager:
    """Insta360 + OBS 整合管理器"""
    
    def __init__(self, config_path: str = None):
        """
        初始化管理器
        
        Args:
            config_path: 配置文件路徑，默認使用 Insta 的 settings.json
        """
        # 載入配置
        self.config = self._load_config(config_path)
        
        # Insta 相關
        self.insta_worker = None
        self.insta_frame_receiver = None
        
        # OBS 相關
        self.obs_video_capture = None
        self.obs_audio_capture = None
        
        # 狀態控制
        self.is_running = False
        self.is_insta_ready = False
        self.is_obs_connected = False
        
        # 圖像處理
        self.image_processor = None
        self.system_monitor = None
        
        # 線程管理
        self.monitor_thread = None
        
    def _load_config(self, config_path: str = None) -> Dict[str, Any]:
        """載入配置文件"""
        if config_path is None:
            config_path = os.path.join(
                os.path.dirname(__file__), 
                'Insta_OpenCV/config/settings.json'
            )
        
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config = json.load(f)
            logger.info(f"已載入配置: {config_path}")
            return config
        except Exception as e:
            logger.error(f"載入配置失敗: {e}")
            return self._get_default_config()
    
    def _get_default_config(self) -> Dict[str, Any]:
        """獲取默認配置"""
        return {
            "insta_ip": "192.168.1.188",
            "http_port": 20000,
            "rtmp_url": "rtmp://192.168.1.188:1935/live/preview",
            "resolution": "3840x1920",
            "framerate": 30,
            "record_audio": True
        }
    
    def start_insta_worker(self, mode: str = "preview") -> bool:
        """
        啟動 Insta Worker
        
        Args:
            mode: "preview" 或 "live"
        """
        try:
            logger.info(f"啟動 Insta Worker ({mode} 模式)...")
            
            # 創建 Insta Worker
            insta_ip = self.config.get("insta_ip", "192.168.1.188")
            self.insta_worker = InstaWorker(ip_address=insta_ip)
            
            # 根據模式啟動
            if mode == "live":
                ready_event = self.insta_worker.start_live_all()
            else:
                ready_event = self.insta_worker.start_preview_all()
            
            # 等待就緒
            logger.info("等待 Insta Worker 就緒...")
            ready_event.wait(timeout=30)  # 等待最多30秒
            
            if ready_event.is_set():
                self.is_insta_ready = True
                logger.info("✓ Insta Worker 已就緒")
                
                # 等待一下讓串流穩定
                time.sleep(3)
                return True
            else:
                logger.error("✗ Insta Worker 啟動超時")
                return False
                
        except Exception as e:
            logger.error(f"啟動 Insta Worker 失敗: {e}")
            return False
    
    def start_obs_capture(self) -> bool:
        """啟動 OBS 串流捕獲"""
        try:
            rtmp_url = self.config.get("rtmp_url", "rtmp://192.168.1.188:1935/live/preview")
            logger.info(f"啟動 OBS 串流捕獲: {rtmp_url}")
            
            # 創建視頻捕獲器
            self.obs_video_capture = OBSStreamCapture(
                stream_url=rtmp_url,
                capture_audio=False  # 音頻單獨處理
            )
            
            # 創建音頻捕獲器
            if self.config.get("record_audio", True):
                self.obs_audio_capture = OBSAudioCapture(
                    stream_url=rtmp_url,
                    sample_rate=44100,
                    channels=2
                )
            
            # 啟動視頻捕獲
            if self.obs_video_capture.start():
                self.is_obs_connected = True
                logger.info("✓ OBS 視頻捕獲已啟動")
                
                # 啟動音頻捕獲
                if self.obs_audio_capture:
                    if self.obs_audio_capture.start_capture():
                        logger.info("✓ OBS 音頻捕獲已啟動")
                    else:
                        logger.warning("⚠ OBS 音頻捕獲啟動失敗")
                
                return True
            else:
                logger.error("✗ OBS 視頻捕獲啟動失敗")
                return False
                
        except Exception as e:
            logger.error(f"啟動 OBS 捕獲失敗: {e}")
            return False
    
    def start_insta_frame_receiver(self) -> bool:
        """啟動 Insta 原生幀接收器 (備用)"""
        try:
            logger.info("啟動 Insta 原生幀接收器...")
            
            self.insta_frame_receiver = FrameReceiver(
                stream_url=self.config.get("rtmp_url"),
                width=3840,
                height=1920
            )
            
            self.insta_frame_receiver.start()
            logger.info("✓ Insta 幀接收器已啟動")
            return True
            
        except Exception as e:
            logger.error(f"啟動 Insta 幀接收器失敗: {e}")
            return False
    
    def start_services(self) -> bool:
        """啟動所有服務"""
        try:
            logger.info("=== 啟動 Insta + OBS 整合服務 ===")
            
            # 初始化圖像處理和系統監控
            self.image_processor = ImageProcessingService()
            self.system_monitor = get_global_system_monitor()
            
            # 1. 先啟動 Insta Worker
            if not self.start_insta_worker("preview"):
                logger.error("Insta Worker 啟動失敗")
                return False
            
            # 2. 啟動 OBS 捕獲
            if not self.start_obs_capture():
                logger.warning("OBS 捕獲啟動失敗，嘗試使用 Insta 原生接收器")
                # 備用方案：使用 Insta 原生幀接收器
                if not self.start_insta_frame_receiver():
                    logger.error("所有視頻源都啟動失敗")
                    return False
            
            self.is_running = True
            
            # 啟動監控線程
            self.monitor_thread = threading.Thread(target=self._monitor_services, daemon=True)
            self.monitor_thread.start()
            
            logger.info("✓ 所有服務已啟動")
            return True
            
        except Exception as e:
            logger.error(f"啟動服務失敗: {e}")
            return False
    
    def stop_services(self):
        """停止所有服務"""
        logger.info("正在停止所有服務...")
        
        self.is_running = False
        
        # 停止 OBS 捕獲
        if self.obs_video_capture:
            self.obs_video_capture.stop()
            self.obs_video_capture = None
        
        if self.obs_audio_capture:
            self.obs_audio_capture.stop_capture()
            self.obs_audio_capture = None
        
        # 停止 Insta 幀接收器
        if self.insta_frame_receiver:
            self.insta_frame_receiver.stop()
            self.insta_frame_receiver = None
        
        # 停止 Insta Worker
        if self.insta_worker:
            self.insta_worker.stop_all()
            self.insta_worker = None
        
        self.is_insta_ready = False
        self.is_obs_connected = False
        
        logger.info("✓ 所有服務已停止")
    
    def get_latest_frame(self) -> Optional[np.ndarray]:
        """獲取最新視頻幀"""
        # 優先使用 OBS 捕獲
        if self.obs_video_capture and self.is_obs_connected:
            frame = self.obs_video_capture.get_latest_frame()
            if frame is not None:
                return frame
        
        # 備用：使用 Insta 原生接收器
        if self.insta_frame_receiver:
            frame = self.insta_frame_receiver.get_latest_frame()
            if frame is not None:
                return frame
        
        return None
    
    def get_audio_data(self) -> Optional[bytes]:
        """獲取最新音頻數據"""
        if self.obs_audio_capture:
            return self.obs_audio_capture.get_audio_data()
        return None
    
    def get_audio_level(self) -> float:
        """獲取音頻音量級別"""
        if self.obs_audio_capture:
            return self.obs_audio_capture.get_audio_level()
        return 0.0
    
    def process_frame(self, frame: np.ndarray) -> np.ndarray:
        """處理視頻幀"""
        if self.image_processor and frame is not None:
            return self.image_processor.process_frame(frame)
        return frame
    
    def _monitor_services(self):
        """服務監控線程"""
        while self.is_running:
            try:
                # 監控 OBS 連接狀態
                if self.obs_video_capture:
                    if not self.obs_video_capture.is_connected:
                        logger.warning("OBS 視頻連接丟失，嘗試重連...")
                        # 這裡可以添加重連邏輯
                
                # 監控系統資源
                if self.system_monitor:
                    # 這裡可以添加系統監控邏輯
                    pass
                
                time.sleep(5)  # 每5秒檢查一次
                
            except Exception as e:
                logger.error(f"服務監控錯誤: {e}")
                time.sleep(1)
    
    def get_status(self) -> Dict[str, Any]:
        """獲取服務狀態"""
        status = {
            "is_running": self.is_running,
            "is_insta_ready": self.is_insta_ready,
            "is_obs_connected": self.is_obs_connected,
            "insta_worker": self.insta_worker is not None,
            "obs_video_capture": self.obs_video_capture is not None,
            "obs_audio_capture": self.obs_audio_capture is not None,
            "insta_frame_receiver": self.insta_frame_receiver is not None
        }
        
        # 添加詳細統計
        if self.obs_video_capture:
            status["obs_video_stats"] = self.obs_video_capture.get_stats()
        
        if self.obs_audio_capture:
            status["obs_audio_stats"] = self.obs_audio_capture.get_stats()
        
        return status
    
    def save_frame(self, frame: np.ndarray, filename: str = None) -> str:
        """保存當前幀"""
        if frame is None:
            raise ValueError("沒有可保存的幀")
        
        if filename is None:
            timestamp = int(time.time())
            filename = f"insta_obs_frame_{timestamp}.jpg"
        
        cv2.imwrite(filename, frame)
        logger.info(f"已保存幀: {filename}")
        return filename
    
    def start_recording(self, output_path: str = None) -> bool:
        """開始錄製"""
        if output_path is None:
            timestamp = int(time.time())
            output_path = f"insta_obs_recording_{timestamp}.mp4"
        
        # 這裡可以實現錄製功能
        logger.info(f"開始錄製到: {output_path}")
        return True


def main_demo():
    """主演示函數"""
    
    # 創建管理器
    manager = InstaOBSManager()
    
    try:
        # 啟動所有服務
        if not manager.start_services():
            logger.error("服務啟動失敗")
            return
        
        # 創建顯示窗口
        cv2.namedWindow('Insta + OBS Stream', cv2.WINDOW_RESIZABLE)
        
        logger.info("=== Insta + OBS 串流已啟動 ===")
        logger.info("控制鍵:")
        logger.info("  'q': 退出")
        logger.info("  's': 保存當前幀")
        logger.info("  'i': 顯示狀態信息")
        logger.info("  'r': 開始/停止錄製")
        
        frame_count = 0
        start_time = time.time()
        recording = False
        
        while True:
            # 獲取最新幀
            frame = manager.get_latest_frame()
            
            if frame is not None:
                frame_count += 1
                
                # 處理幀 (可選)
                # processed_frame = manager.process_frame(frame)
                processed_frame = frame
                
                # 添加信息到幀上
                current_time = time.time()
                fps = frame_count / max(current_time - start_time, 1)
                
                # 顯示信息
                info_text = f"FPS: {fps:.1f}"
                cv2.putText(processed_frame, info_text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # 音頻音量級別
                audio_level = manager.get_audio_level()
                volume_text = f"Audio: {audio_level:.2f}"
                cv2.putText(processed_frame, volume_text, (10, 70), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                
                # 錄製狀態
                if recording:
                    cv2.putText(processed_frame, "REC", (processed_frame.shape[1] - 100, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                
                # 顯示幀
                cv2.imshow('Insta + OBS Stream', processed_frame)
            else:
                # 顯示等待畫面
                waiting_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(waiting_frame, "Waiting for stream...", (50, 240), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imshow('Insta + OBS Stream', waiting_frame)
            
            # 處理按鍵
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('s') and frame is not None:
                # 保存當前幀
                try:
                    filename = manager.save_frame(frame)
                    logger.info(f"已保存: {filename}")
                except Exception as e:
                    logger.error(f"保存失敗: {e}")
            elif key == ord('i'):
                # 顯示狀態信息
                status = manager.get_status()
                logger.info("=== 服務狀態 ===")
                for key, value in status.items():
                    logger.info(f"  {key}: {value}")
            elif key == ord('r'):
                # 切換錄製狀態
                recording = not recording
                if recording:
                    logger.info("開始錄製")
                    # manager.start_recording()
                else:
                    logger.info("停止錄製")
            
            # 定期重置計數器
            if frame_count > 1000:
                frame_count = 0
                start_time = time.time()
    
    except KeyboardInterrupt:
        logger.info("收到中斷信號")
    
    finally:
        # 清理資源
        cv2.destroyAllWindows()
        manager.stop_services()
        logger.info("程序結束")


if __name__ == "__main__":
    main_demo()
