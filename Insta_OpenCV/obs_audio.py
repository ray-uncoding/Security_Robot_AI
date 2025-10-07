#!/usr/bin/env python3
"""
OBS 音頻捕獲專用模組
OBS Audio Capture Module

專門處理從 OBS 串流中提取和處理音頻數據
"""

import subprocess
import threading
import queue
import time
import wave
import numpy as np
import logging
from typing import Optional, Callable, Dict, Any
import os

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class OBSAudioCapture:
    """OBS 音頻捕獲類"""
    
    def __init__(self, 
                 stream_url: str,
                 sample_rate: int = 44100,
                 channels: int = 2,
                 output_format: str = "wav"):
        """
        初始化音頻捕獲器
        
        Args:
            stream_url: OBS 串流 URL
            sample_rate: 採樣率 (Hz)
            channels: 聲道數 (1=單聲道, 2=立體聲)
            output_format: 輸出格式 ("wav", "raw", "mp3")
        """
        self.stream_url = stream_url
        self.sample_rate = sample_rate
        self.channels = channels
        self.output_format = output_format
        
        # 狀態控制
        self.is_running = False
        self.is_recording = False
        
        # 音頻處理
        self.audio_process = None
        self.audio_thread = None
        self.audio_queue = queue.Queue(maxsize=1000)
        
        # 錄音控制
        self.wav_writer = None
        self.output_file = None
        
        # 回調函數
        self.audio_callback = None
        
        # 統計信息
        self.bytes_received = 0
        self.start_time = None
    
    def set_audio_callback(self, callback: Callable[[bytes], None]):
        """設置音頻數據回調函數"""
        self.audio_callback = callback
    
    def start_capture(self) -> bool:
        """開始音頻捕獲"""
        if self.is_running:
            logger.warning("音頻捕獲已在運行中")
            return True
        
        try:
            # 構建 FFmpeg 命令
            ffmpeg_cmd = self._build_ffmpeg_command()
            
            logger.info(f"啟動音頻捕獲: {' '.join(ffmpeg_cmd)}")
            
            # 啟動 FFmpeg 進程
            self.audio_process = subprocess.Popen(
                ffmpeg_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0
            )
            
            self.is_running = True
            self.start_time = time.time()
            self.bytes_received = 0
            
            # 啟動音頻讀取線程
            self.audio_thread = threading.Thread(target=self._audio_worker, daemon=True)
            self.audio_thread.start()
            
            logger.info("音頻捕獲已啟動")
            return True
            
        except Exception as e:
            logger.error(f"啟動音頻捕獲失敗: {e}")
            return False
    
    def stop_capture(self):
        """停止音頻捕獲"""
        logger.info("正在停止音頻捕獲...")
        
        self.is_running = False
        
        # 停止錄音
        if self.is_recording:
            self.stop_recording()
        
        # 終止 FFmpeg 進程
        if self.audio_process:
            try:
                self.audio_process.terminate()
                self.audio_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.audio_process.kill()
            except Exception as e:
                logger.warning(f"停止 FFmpeg 進程時出錯: {e}")
            finally:
                self.audio_process = None
        
        # 等待線程結束
        if self.audio_thread and self.audio_thread.is_alive():
            self.audio_thread.join(timeout=2)
        
        logger.info("音頻捕獲已停止")
    
    def start_recording(self, output_file: str) -> bool:
        """開始錄音到文件"""
        if self.is_recording:
            logger.warning("已在錄音中")
            return False
        
        if not self.is_running:
            logger.error("請先啟動音頻捕獲")
            return False
        
        try:
            self.output_file = output_file
            
            if self.output_format.lower() == "wav":
                self.wav_writer = wave.open(output_file, 'wb')
                self.wav_writer.setnchannels(self.channels)
                self.wav_writer.setsampwidth(2)  # 16-bit
                self.wav_writer.setframerate(self.sample_rate)
            
            self.is_recording = True
            logger.info(f"開始錄音到: {output_file}")
            return True
            
        except Exception as e:
            logger.error(f"開始錄音失敗: {e}")
            return False
    
    def stop_recording(self):
        """停止錄音"""
        if not self.is_recording:
            return
        
        self.is_recording = False
        
        if self.wav_writer:
            try:
                self.wav_writer.close()
                self.wav_writer = None
                logger.info(f"錄音已保存到: {self.output_file}")
            except Exception as e:
                logger.error(f"保存錄音文件失敗: {e}")
        
        self.output_file = None
    
    def get_audio_data(self) -> Optional[bytes]:
        """獲取音頻數據"""
        try:
            return self.audio_queue.get_nowait()
        except queue.Empty:
            return None
    
    def get_audio_level(self) -> float:
        """獲取音頻音量級別 (0-1)"""
        try:
            audio_data = self.audio_queue.get_nowait()
            if audio_data:
                # 將字節數據轉換為 numpy 數組
                audio_array = np.frombuffer(audio_data, dtype=np.int16)
                # 計算 RMS (均方根) 音量
                rms = np.sqrt(np.mean(audio_array.astype(np.float32) ** 2))
                # 歸一化到 0-1 範圍
                normalized = rms / 32768.0
                return min(1.0, normalized)
        except (queue.Empty, Exception):
            pass
        return 0.0
    
    def _build_ffmpeg_command(self) -> list:
        """構建 FFmpeg 命令"""
        cmd = [
            'ffmpeg',
            '-i', self.stream_url,
            '-vn',  # 不處理視頻
            '-acodec', 'pcm_s16le',  # 16-bit PCM
            '-ac', str(self.channels),  # 聲道數
            '-ar', str(self.sample_rate),  # 採樣率
            '-f', 'wav' if self.output_format == 'wav' else 's16le',
            '-'  # 輸出到 stdout
        ]
        return cmd
    
    def _audio_worker(self):
        """音頻處理工作線程"""
        chunk_size = 4096
        
        while self.is_running and self.audio_process:
            try:
                # 讀取音頻數據
                audio_data = self.audio_process.stdout.read(chunk_size)
                
                if not audio_data:
                    logger.warning("音頻流結束")
                    break
                
                self.bytes_received += len(audio_data)
                
                # 將數據放入隊列
                try:
                    self.audio_queue.put_nowait(audio_data)
                except queue.Full:
                    # 隊列滿時丟棄最舊的數據
                    try:
                        self.audio_queue.get_nowait()
                        self.audio_queue.put_nowait(audio_data)
                    except queue.Empty:
                        pass
                
                # 如果正在錄音，寫入文件
                if self.is_recording and self.wav_writer:
                    try:
                        self.wav_writer.writeframes(audio_data)
                    except Exception as e:
                        logger.error(f"寫入音頻文件失敗: {e}")
                
                # 調用回調函數
                if self.audio_callback:
                    try:
                        self.audio_callback(audio_data)
                    except Exception as e:
                        logger.error(f"音頻回調函數出錯: {e}")
                
            except Exception as e:
                logger.error(f"音頻處理錯誤: {e}")
                time.sleep(0.1)
        
        logger.info("音頻工作線程結束")
    
    def get_stats(self) -> Dict[str, Any]:
        """獲取統計信息"""
        duration = (time.time() - self.start_time) if self.start_time else 0
        
        return {
            'is_running': self.is_running,
            'is_recording': self.is_recording,
            'stream_url': self.stream_url,
            'sample_rate': self.sample_rate,
            'channels': self.channels,
            'bytes_received': self.bytes_received,
            'duration_seconds': round(duration, 2),
            'queue_size': self.audio_queue.qsize(),
            'bitrate_kbps': round((self.bytes_received * 8 / 1000) / max(duration, 1), 2) if duration > 0 else 0
        }


def audio_level_monitor_example():
    """音頻音量監控示例"""
    
    OBS_URL = "rtmp://localhost:1935/live/stream"  # 修改為你的 OBS URL
    
    # 創建音頻捕獲器
    audio_capture = OBSAudioCapture(OBS_URL)
    
    # 音量級別可視化
    def print_audio_level(level):
        bar_length = 50
        filled = int(level * bar_length)
        bar = '█' * filled + '░' * (bar_length - filled)
        percentage = level * 100
        print(f"\r音量: |{bar}| {percentage:5.1f}%", end='', flush=True)
    
    # 啟動捕獲
    if not audio_capture.start_capture():
        logger.error("無法啟動音頻捕獲")
        return
    
    logger.info("音頻監控已啟動，按 Ctrl+C 停止")
    
    try:
        while True:
            level = audio_capture.get_audio_level()
            print_audio_level(level)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n正在停止...")
    
    finally:
        audio_capture.stop_capture()
        
        # 顯示統計信息
        stats = audio_capture.get_stats()
        print("\n=== 音頻統計 ===")
        for key, value in stats.items():
            print(f"{key}: {value}")

def audio_recording_example():
    """音頻錄製示例"""
    
    OBS_URL = "rtmp://localhost:1935/live/stream"  # 修改為你的 OBS URL
    
    # 創建音頻捕獲器
    audio_capture = OBSAudioCapture(
        stream_url=OBS_URL,
        sample_rate=44100,
        channels=2
    )
    
    # 啟動捕獲
    if not audio_capture.start_capture():
        logger.error("無法啟動音頻捕獲")
        return
    
    # 生成錄音文件名
    timestamp = int(time.time())
    output_file = f"obs_audio_{timestamp}.wav"
    
    # 開始錄音
    if audio_capture.start_recording(output_file):
        logger.info(f"錄音到: {output_file}")
        logger.info("錄音中... 按 Enter 停止錄音")
        
        try:
            input()  # 等待用戶按 Enter
        except KeyboardInterrupt:
            pass
        
        # 停止錄音
        audio_capture.stop_recording()
    
    # 停止捕獲
    audio_capture.stop_capture()
    
    # 顯示統計信息
    stats = audio_capture.get_stats()
    logger.info("=== 錄音完成 ===")
    for key, value in stats.items():
        logger.info(f"{key}: {value}")

def main():
    """主函數"""
    import sys
    
    if len(sys.argv) > 1:
        mode = sys.argv[1]
        if mode == "monitor":
            audio_level_monitor_example()
        elif mode == "record":
            audio_recording_example()
        else:
            print("未知模式:", mode)
    else:
        print("OBS 音頻捕獲示例")
        print("用法:")
        print("  python obs_audio.py monitor  # 音量監控")
        print("  python obs_audio.py record   # 音頻錄製")

if __name__ == "__main__":
    main()
