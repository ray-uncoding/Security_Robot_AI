#!/usr/bin/env python3
"""
快速啟動 Insta Worker + OBS 串流捕獲
Quick start script for Insta Worker + OBS stream capture
"""

import os
import sys
import time
import cv2
import logging
import threading
import numpy as np

# 添加專案路徑
project_root = os.path.dirname(os.path.abspath(__file__))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from Insta_OpenCV.controller.insta_worker import InstaWorker
from Insta_OpenCV.obs_stream_capture import OBSStreamCapture
from obs_audio import OBSAudioCapture

# 設置日誌
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def quick_start(insta_ip="192.168.1.188", rtmp_url="rtmp://192.168.1.188:1935/live/preview"):
    """快速啟動流程"""
    
    logger.info("=== 開始啟動 Insta Worker + OBS 捕獲 ===")
    
    # 步驟 1: 啟動 Insta Worker
    logger.info("1. 啟動 Insta Worker...")
    insta_worker = InstaWorker(ip_address=insta_ip)
    
    # 啟動預覽模式
    ready_event = insta_worker.start_preview_all()
    
    logger.info("   等待 Insta Worker 就緒...")
    if ready_event.wait(timeout=30):
        logger.info("   ✓ Insta Worker 已就緒")
    else:
        logger.error("   ✗ Insta Worker 啟動超時")
        return False
    
    # 等待串流穩定
    logger.info("   等待串流穩定...")
    time.sleep(5)
    
    # 步驟 2: 啟動 OBS 視頻捕獲
    logger.info("2. 啟動 OBS 視頻捕獲...")
    obs_video = OBSStreamCapture(stream_url=rtmp_url, capture_audio=False)
    
    if obs_video.start():
        logger.info("   ✓ OBS 視頻捕獲已啟動")
    else:
        logger.error("   ✗ OBS 視頻捕獲啟動失敗")
        insta_worker.stop_all()
        return False
    
    # 步驟 3: 啟動 OBS 音頻捕獲
    logger.info("3. 啟動 OBS 音頻捕獲...")
    obs_audio = OBSAudioCapture(stream_url=rtmp_url)
    
    if obs_audio.start_capture():
        logger.info("   ✓ OBS 音頻捕獲已啟動")
    else:
        logger.warning("   ⚠ OBS 音頻捕獲啟動失敗")
    
    # 步驟 4: 創建顯示窗口
    logger.info("4. 開始顯示串流...")
    cv2.namedWindow('Insta Stream via OBS', cv2.WINDOW_NORMAL)
    
    logger.info("=== 系統已就緒 ===")
    logger.info("按鍵控制:")
    logger.info("  'q': 退出")
    logger.info("  's': 保存當前幀 (原始分辨率)")
    logger.info("  'a': 開始/停止音頻錄製")
    logger.info("  'i': 顯示統計信息")
    logger.info("  '+': 增大顯示尺寸")
    logger.info("  '-': 減小顯示尺寸")
    logger.info("  'r': 重置顯示尺寸")
    
    # 統計變數
    frame_count = 0
    start_time = time.time()
    audio_recording = False
    display_scale = 0.33  # 初始縮放比例 (3840 -> 1280)
    fps = 0.0  # 初始化 FPS
    
    try:
        while True:
            # 獲取視頻幀
            frame = obs_video.get_latest_frame()
            
            if frame is not None:
                frame_count += 1
                
                # 縮放畫面到合適的顯示大小 (原本 3840x1920 太大)
                original_height, original_width = frame.shape[:2]
                
                # 使用動態縮放比例
                target_width = int(original_width * display_scale)
                target_height = int(original_height * display_scale)
                
                # 縮放畫面
                display_frame = cv2.resize(frame, (target_width, target_height), 
                                         interpolation=cv2.INTER_LINEAR)
                
                # 計算 FPS
                current_time = time.time()
                elapsed = current_time - start_time
                if elapsed > 1.0:
                    fps = frame_count / elapsed
                    frame_count = 0
                    start_time = current_time
                
                # 調整文字大小以適應縮放後的畫面
                font_scale = display_scale * 0.8
                thickness = max(1, int(display_scale * 6))
                
                # 顯示 FPS
                fps_text = f"FPS: {fps:.1f}"
                cv2.putText(display_frame, fps_text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 255, 0), thickness)
                
                # 顯示原始分辨率信息
                res_text = f"Original: {original_width}x{original_height}"
                cv2.putText(display_frame, res_text, (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.6, (255, 255, 255), thickness)
                
                # 顯示音頻音量
                if obs_audio:
                    audio_level = obs_audio.get_audio_level()
                    volume_text = f"Audio: {audio_level:.2f}"
                    cv2.putText(display_frame, volume_text, (10, 90), 
                               cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 255, 255), thickness)
                
                # 錄音狀態指示
                if audio_recording:
                    cv2.putText(display_frame, "REC", (display_frame.shape[1] - 100, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 255), thickness + 1)
                
                # 顯示縮放後的畫面
                cv2.imshow('Insta Stream via OBS', display_frame)
            else:
                # 等待畫面
                waiting_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(waiting_frame, "Waiting for Insta stream...", (50, 240), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imshow('Insta Stream via OBS', waiting_frame)
            
            # 處理按鍵
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('s') and frame is not None:
                # 保存當前幀 (保存原始大小的幀)
                timestamp = int(time.time())
                filename = f"insta_frame_{timestamp}.jpg"
                cv2.imwrite(filename, frame)  # 保存原始 frame，不是 display_frame
                logger.info(f"已保存幀: {filename} (原始分辨率: {frame.shape[1]}x{frame.shape[0]})")
            elif key == ord('a'):
                # 切換音頻錄製
                if not audio_recording:
                    timestamp = int(time.time())
                    audio_file = f"insta_audio_{timestamp}.wav"
                    if obs_audio and obs_audio.start_recording(audio_file):
                        audio_recording = True
                        logger.info(f"開始錄音: {audio_file}")
                else:
                    if obs_audio:
                        obs_audio.stop_recording()
                        audio_recording = False
                        logger.info("錄音已停止")
            elif key == ord('i'):
                # 顯示統計信息
                logger.info("=== 系統統計 ===")
                
                # OBS 視頻統計
                video_stats = obs_video.get_stats()
                logger.info("OBS 視頻:")
                for k, v in video_stats.items():
                    logger.info(f"  {k}: {v}")
                
                # OBS 音頻統計
                if obs_audio:
                    audio_stats = obs_audio.get_stats()
                    logger.info("OBS 音頻:")
                    for k, v in audio_stats.items():
                        logger.info(f"  {k}: {v}")
            elif key == ord('+') or key == ord('='):
                # 增大顯示尺寸
                display_scale = min(1.0, display_scale + 0.1)
                logger.info(f"顯示縮放: {display_scale:.1f}")
            elif key == ord('-'):
                # 減小顯示尺寸
                display_scale = max(0.1, display_scale - 0.1)
                logger.info(f"顯示縮放: {display_scale:.1f}")
            elif key == ord('r'):
                # 重置顯示尺寸
                display_scale = 0.33
                logger.info(f"顯示縮放重置: {display_scale:.1f}")
    
    except KeyboardInterrupt:
        logger.info("收到中斷信號")
    
    finally:
        # 清理資源
        logger.info("正在清理資源...")
        
        cv2.destroyAllWindows()
        
        if obs_audio:
            if audio_recording:
                obs_audio.stop_recording()
            obs_audio.stop_capture()
        
        obs_video.stop()
        insta_worker.stop_all()
        
        logger.info("✓ 資源清理完成")

def test_connections():
    """測試連接功能"""
    logger.info("=== 連接測試 ===")
    
    # 測試 RTMP 連接
    test_url = "rtmp://192.168.1.188:1935/live/preview"
    logger.info(f"測試 RTMP 連接: {test_url}")
    
    cap = cv2.VideoCapture(test_url)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret and frame is not None:
            logger.info("✓ RTMP 連接成功")
            logger.info(f"  分辨率: {frame.shape[1]}x{frame.shape[0]}")
        else:
            logger.warning("⚠ RTMP 連接成功但無法讀取幀")
    else:
        logger.error("✗ RTMP 連接失敗")
    cap.release()

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Insta Worker + OBS 啟動工具")
    parser.add_argument("--test", action="store_true", help="僅測試連接")
    parser.add_argument("--ip", default="192.168.1.188", help="Insta360 IP 地址")
    parser.add_argument("--rtmp", default="rtmp://192.168.1.188:1935/live/preview", 
                       help="RTMP 串流 URL")
    
    args = parser.parse_args()
    
    if args.test:
        test_connections()
    else:
        logger.info(f"Insta IP: {args.ip}")
        logger.info(f"RTMP URL: {args.rtmp}")
        
        quick_start(args.ip, args.rtmp)
