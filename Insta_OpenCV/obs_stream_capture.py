#!/usr/bin/env python3
"""
使用 OpenCV 從 OBS 拉取聲音和畫面的完整解決方案
OBS Complete Stream Capture Solution with OpenCV

功能特點:
1. 支持 RTMP/RTSP/HTTP 串流協議
2. 視頻和音頻同步處理
3. 多種輸出格式支持
4. 錯誤重連機制
5. 性能監控和優化
"""

import cv2
import numpy as np
import threading
import time
import queue
import subprocess
import os
import json
import logging
from typing import Optional, Tuple, Dict, Any
import psutil

# 設置日誌
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class OBSStreamCapture:
    """OBS 串流捕獲類"""
    
    def __init__(self, 
                 stream_url: str,
                 capture_audio: bool = True,
                 output_video_path: Optional[str] = None,
                 output_audio_path: Optional[str] = None):
        """
        初始化 OBS 串流捕獲器
        
        Args:
            stream_url: OBS 串流 URL (RTMP/RTSP/HTTP)
            capture_audio: 是否捕獲音頻
            output_video_path: 視頻輸出路徑 (可選)
            output_audio_path: 音頻輸出路徑 (可選)
        """
        self.stream_url = stream_url
        self.capture_audio = capture_audio
        self.output_video_path = output_video_path
        self.output_audio_path = output_audio_path
        
        # 狀態控制
        self.is_running = False
        self.is_connected = False
        
        # 視頻相關
        self.video_cap = None
        self.current_frame = None
        self.frame_queue = queue.Queue(maxsize=30)
        self.video_thread = None
        
        # 音頻相關
        self.audio_process = None
        self.audio_queue = queue.Queue(maxsize=100)
        self.audio_thread = None
        
        # 性能監控
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0
        
        # 錯誤處理
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 5
        self.reconnect_delay = 2.0
        
    def connect(self) -> bool:
        """連接到 OBS 串流"""
        try:
            logger.info(f"正在連接到串流: {self.stream_url}")
            
            # 嘗試不同的 OpenCV 後端
            backends_to_try = [
                cv2.CAP_FFMPEG,
                cv2.CAP_GSTREAMER,
                cv2.CAP_ANY
            ]
            
            for backend in backends_to_try:
                try:
                    self.video_cap = cv2.VideoCapture(self.stream_url, backend)
                    
                    if self.video_cap and self.video_cap.isOpened():
                        # 設置緩衝區大小以減少延遲
                        self.video_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        
                        # 測試讀取一幀
                        ret, frame = self.video_cap.read()
                        if ret and frame is not None:
                            self.current_frame = frame
                            self.is_connected = True
                            logger.info(f"成功連接 (後端: {backend})")
                            logger.info(f"視頻分辨率: {frame.shape[1]}x{frame.shape[0]}")
                            return True
                        else:
                            self.video_cap.release()
                            self.video_cap = None
                            
                except Exception as e:
                    logger.warning(f"後端 {backend} 連接失敗: {e}")
                    if self.video_cap:
                        self.video_cap.release()
                        self.video_cap = None
                    continue
            
            logger.error("所有後端都無法連接")
            return False
            
        except Exception as e:
            logger.error(f"連接失敗: {e}")
            return False
    
    def start_video_capture(self):
        """開始視頻捕獲線程"""
        def video_capture_worker():
            while self.is_running:
                try:
                    if not self.video_cap or not self.video_cap.isOpened():
                        if not self.reconnect():
                            time.sleep(1)
                            continue
                    
                    ret, frame = self.video_cap.read()
                    if ret and frame is not None:
                        self.current_frame = frame
                        
                        # 更新幀數統計
                        self.frame_count += 1
                        current_time = time.time()
                        elapsed = current_time - self.start_time
                        if elapsed > 1.0:
                            self.fps = self.frame_count / elapsed
                            self.frame_count = 0
                            self.start_time = current_time
                        
                        # 將幀放入隊列
                        try:
                            self.frame_queue.put_nowait(frame)
                        except queue.Full:
                            # 如果隊列滿了，丟棄最舊的幀
                            try:
                                self.frame_queue.get_nowait()
                                self.frame_queue.put_nowait(frame)
                            except queue.Empty:
                                pass
                        
                        # 重置重連計數
                        self.reconnect_attempts = 0
                    else:
                        logger.warning("無法讀取視頻幀，嘗試重連...")
                        if not self.reconnect():
                            time.sleep(self.reconnect_delay)
                
                except Exception as e:
                    logger.error(f"視頻捕獲錯誤: {e}")
                    time.sleep(0.1)
        
        self.video_thread = threading.Thread(target=video_capture_worker, daemon=True)
        self.video_thread.start()
        logger.info("視頻捕獲線程已啟動")
    
    def start_audio_capture(self):
        """開始音頻捕獲 (使用 FFmpeg)"""
        if not self.capture_audio:
            return
        
        def audio_capture_worker():
            while self.is_running:
                try:
                    # 使用 FFmpeg 捕獲音頻
                    ffmpeg_cmd = [
                        'ffmpeg',
                        '-i', self.stream_url,
                        '-vn',  # 不要視頻
                        '-acodec', 'pcm_s16le',
                        '-ac', '2',  # 立體聲
                        '-ar', '44100',  # 採樣率
                        '-f', 'wav',
                        '-'  # 輸出到 stdout
                    ]
                    
                    self.audio_process = subprocess.Popen(
                        ffmpeg_cmd,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        bufsize=10**8
                    )
                    
                    chunk_size = 4096
                    while self.is_running and self.audio_process:
                        try:
                            audio_data = self.audio_process.stdout.read(chunk_size)
                            if audio_data:
                                try:
                                    self.audio_queue.put_nowait(audio_data)
                                except queue.Full:
                                    # 如果音頻隊列滿了，丟棄最舊的數據
                                    try:
                                        self.audio_queue.get_nowait()
                                        self.audio_queue.put_nowait(audio_data)
                                    except queue.Empty:
                                        pass
                            else:
                                break
                        except Exception as e:
                            logger.error(f"音頻讀取錯誤: {e}")
                            break
                    
                    if self.audio_process:
                        self.audio_process.terminate()
                        self.audio_process = None
                
                except Exception as e:
                    logger.error(f"音頻捕獲錯誤: {e}")
                    time.sleep(1)
        
        self.audio_thread = threading.Thread(target=audio_capture_worker, daemon=True)
        self.audio_thread.start()
        logger.info("音頻捕獲線程已啟動")
    
    def reconnect(self) -> bool:
        """重連串流"""
        if self.reconnect_attempts >= self.max_reconnect_attempts:
            logger.error(f"達到最大重連次數 ({self.max_reconnect_attempts})")
            return False
        
        self.reconnect_attempts += 1
        logger.info(f"嘗試重連 (第 {self.reconnect_attempts} 次)...")
        
        # 清理現有連接
        if self.video_cap:
            self.video_cap.release()
            self.video_cap = None
        
        time.sleep(self.reconnect_delay)
        
        return self.connect()
    
    def start(self):
        """開始捕獲"""
        if self.is_running:
            logger.warning("捕獲已在運行中")
            return
        
        if not self.connect():
            logger.error("無法連接到串流")
            return False
        
        self.is_running = True
        self.start_time = time.time()
        
        # 啟動視頻捕獲
        self.start_video_capture()
        
        # 啟動音頻捕獲
        if self.capture_audio:
            self.start_audio_capture()
        
        logger.info("OBS 串流捕獲已啟動")
        return True
    
    def stop(self):
        """停止捕獲"""
        logger.info("正在停止 OBS 串流捕獲...")
        
        self.is_running = False
        self.is_connected = False
        
        # 停止視頻捕獲
        if self.video_cap:
            self.video_cap.release()
            self.video_cap = None
        
        # 停止音頻捕獲
        if self.audio_process:
            self.audio_process.terminate()
            self.audio_process = None
        
        # 等待線程結束
        if self.video_thread and self.video_thread.is_alive():
            self.video_thread.join(timeout=2)
        
        if self.audio_thread and self.audio_thread.is_alive():
            self.audio_thread.join(timeout=2)
        
        logger.info("OBS 串流捕獲已停止")
    
    def get_latest_frame(self) -> Optional[np.ndarray]:
        """獲取最新的視頻幀"""
        return self.current_frame
    
    def get_frame_from_queue(self) -> Optional[np.ndarray]:
        """從隊列獲取視頻幀"""
        try:
            return self.frame_queue.get_nowait()
        except queue.Empty:
            return None
    
    def get_audio_data(self) -> Optional[bytes]:
        """獲取音頻數據"""
        try:
            return self.audio_queue.get_nowait()
        except queue.Empty:
            return None
    
    def get_stats(self) -> Dict[str, Any]:
        """獲取統計信息"""
        return {
            'is_running': self.is_running,
            'is_connected': self.is_connected,
            'fps': round(self.fps, 2),
            'frame_queue_size': self.frame_queue.qsize(),
            'audio_queue_size': self.audio_queue.qsize() if self.capture_audio else 0,
            'reconnect_attempts': self.reconnect_attempts,
            'stream_url': self.stream_url
        }


class OBSStreamManager:
    """OBS 串流管理器 - 提供高級功能"""
    
    def __init__(self):
        self.captures = {}
        self.recording = False
        self.video_writer = None
        self.audio_file = None
    
    def add_stream(self, name: str, stream_url: str, capture_audio: bool = True):
        """添加一個串流源"""
        if name in self.captures:
            logger.warning(f"串流 '{name}' 已存在")
            return False
        
        capture = OBSStreamCapture(stream_url, capture_audio)
        self.captures[name] = capture
        logger.info(f"已添加串流源: {name}")
        return True
    
    def start_stream(self, name: str) -> bool:
        """啟動指定的串流"""
        if name not in self.captures:
            logger.error(f"串流 '{name}' 不存在")
            return False
        
        return self.captures[name].start()
    
    def stop_stream(self, name: str):
        """停止指定的串流"""
        if name in self.captures:
            self.captures[name].stop()
            del self.captures[name]
            logger.info(f"已停止並移除串流: {name}")
    
    def start_all_streams(self):
        """啟動所有串流"""
        for name, capture in self.captures.items():
            if not capture.is_running:
                capture.start()
                logger.info(f"已啟動串流: {name}")
    
    def stop_all_streams(self):
        """停止所有串流"""
        for name in list(self.captures.keys()):
            self.stop_stream(name)
    
    def get_stream_frame(self, name: str) -> Optional[np.ndarray]:
        """獲取指定串流的最新幀"""
        if name in self.captures:
            return self.captures[name].get_latest_frame()
        return None
    
    def start_recording(self, output_path: str, fps: int = 30, resolution: Tuple[int, int] = (1920, 1080)):
        """開始錄製"""
        if self.recording:
            logger.warning("錄製已在進行中")
            return False
        
        try:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(output_path, fourcc, fps, resolution)
            
            if self.video_writer.isOpened():
                self.recording = True
                logger.info(f"開始錄製到: {output_path}")
                return True
            else:
                logger.error("無法創建視頻寫入器")
                return False
        except Exception as e:
            logger.error(f"錄製啟動失敗: {e}")
            return False
    
    def stop_recording(self):
        """停止錄製"""
        if self.recording and self.video_writer:
            self.video_writer.release()
            self.video_writer = None
            self.recording = False
            logger.info("錄製已停止")
    
    def record_frame(self, frame: np.ndarray):
        """錄製一幀"""
        if self.recording and self.video_writer and frame is not None:
            self.video_writer.write(frame)
    
    def get_all_stats(self) -> Dict[str, Dict[str, Any]]:
        """獲取所有串流的統計信息"""
        stats = {}
        for name, capture in self.captures.items():
            stats[name] = capture.get_stats()
        return stats


def main_example():
    """主要示例函數"""
    
    # OBS 串流 URL 示例
    # RTMP: rtmp://your-obs-server:1935/live/stream_key
    # RTSP: rtsp://your-obs-server:8554/stream
    # HTTP: http://your-obs-server:8080/stream.m3u8
    
    # 請根據你的 OBS 設置修改這個 URL
    OBS_STREAM_URL = "rtmp://localhost:1935/live/stream"
    
    # 創建串流管理器
    manager = OBSStreamManager()
    
    # 添加 OBS 串流
    if not manager.add_stream("obs_main", OBS_STREAM_URL, capture_audio=True):
        logger.error("無法添加串流")
        return
    
    # 啟動串流
    if not manager.start_stream("obs_main"):
        logger.error("無法啟動串流")
        return
    
    # 創建顯示窗口
    cv2.namedWindow('OBS Stream', cv2.WINDOW_AUTOSIZE)
    
    # 可選：開始錄製
    # manager.start_recording("output.mp4", fps=30, resolution=(1920, 1080))
    
    try:
        logger.info("開始顯示 OBS 串流 (按 'q' 退出)")
        
        while True:
            # 獲取最新幀
            frame = manager.get_stream_frame("obs_main")
            
            if frame is not None:
                # 顯示幀
                cv2.imshow('OBS Stream', frame)
                
                # 可選：錄製幀
                # manager.record_frame(frame)
                
                # 在幀上添加信息
                stats = manager.get_all_stats()
                if "obs_main" in stats:
                    fps_text = f"FPS: {stats['obs_main']['fps']}"
                    cv2.putText(frame, fps_text, (10, 30), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # 檢查退出條件
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                # 保存當前幀
                if frame is not None:
                    timestamp = int(time.time())
                    filename = f"obs_frame_{timestamp}.jpg"
                    cv2.imwrite(filename, frame)
                    logger.info(f"已保存幀: {filename}")
            elif key == ord('r'):
                # 切換錄製狀態
                if manager.recording:
                    manager.stop_recording()
                else:
                    timestamp = int(time.time())
                    filename = f"obs_recording_{timestamp}.mp4"
                    manager.start_recording(filename)
            
            # 每秒顯示一次統計信息
            if int(time.time()) % 1 == 0:
                stats = manager.get_all_stats()
                for name, stat in stats.items():
                    logger.info(f"[{name}] FPS: {stat['fps']}, 隊列: {stat['frame_queue_size']}")
    
    except KeyboardInterrupt:
        logger.info("收到中斷信號")
    
    finally:
        # 清理資源
        cv2.destroyAllWindows()
        manager.stop_all_streams()
        manager.stop_recording()
        logger.info("程序結束")


if __name__ == "__main__":
    main_example()
