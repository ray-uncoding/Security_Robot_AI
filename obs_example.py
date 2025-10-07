#!/usr/bin/env python3
"""
OBS 串流捕獲簡化示例
Simple OBS Stream Capture Example

這個示例展示了如何快速開始使用 OpenCV 從 OBS 捕獲視頻和音頻
"""

import cv2
import json
import time
import logging
from Insta_OpenCV.obs_stream_capture import OBSStreamCapture, OBSStreamManager

# 設置日誌
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def load_config(config_path: str = "obs_config.json"):
    """載入配置文件"""
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except FileNotFoundError:
        logger.warning(f"配置文件 {config_path} 不存在，使用默認設置")
        return get_default_config()
    except Exception as e:
        logger.error(f"載入配置文件失敗: {e}")
        return get_default_config()

def get_default_config():
    """獲取默認配置"""
    return {
        "obs_streams": {
            "main_stream": {
                "url": "rtmp://localhost:1935/live/stream",
                "capture_audio": True
            }
        }
    }

def simple_obs_capture():
    """簡單的 OBS 捕獲示例"""
    
    # 你的 OBS 串流 URL - 請根據實際情況修改
    # 常見的 OBS 串流 URL 格式：
    # RTMP: rtmp://localhost:1935/live/your_stream_key
    # RTSP: rtsp://localhost:8554/your_stream
    # HTTP: http://localhost:8080/stream.m3u8
    
    OBS_URL = "rtmp://localhost:1935/live/stream"  # 修改為你的 OBS 串流 URL
    
    logger.info("開始 OBS 串流捕獲示例")
    
    # 創建 OBS 捕獲器
    obs_capture = OBSStreamCapture(
        stream_url=OBS_URL,
        capture_audio=True  # 設為 False 如果只需要視頻
    )
    
    # 啟動捕獲
    if not obs_capture.start():
        logger.error("無法啟動 OBS 串流捕獲")
        return
    
    # 創建顯示窗口
    cv2.namedWindow('OBS Stream', cv2.WINDOW_RESIZABLE)
    
    logger.info("OBS 串流已開始，按以下鍵：")
    logger.info("  'q': 退出")
    logger.info("  's': 保存當前幀")
    logger.info("  'i': 顯示統計信息")
    
    frame_count = 0
    start_time = time.time()
    
    try:
        while True:
            # 獲取最新幀
            frame = obs_capture.get_latest_frame()
            
            if frame is not None:
                frame_count += 1
                
                # 添加 FPS 信息到幀上
                current_time = time.time()
                elapsed = current_time - start_time
                if elapsed > 1.0:
                    fps = frame_count / elapsed
                    frame_count = 0
                    start_time = current_time
                    
                    # 在幀上顯示 FPS
                    fps_text = f"FPS: {fps:.1f}"
                    cv2.putText(frame, fps_text, (10, 30), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # 顯示幀
                cv2.imshow('OBS Stream', frame)
            else:
                # 如果沒有幀，顯示等待信息
                waiting_frame = cv2.imread("data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNkYPhfDwAChwGA60e6kgAAAABJRU5ErkJggg==", cv2.IMREAD_COLOR)
                if waiting_frame is None:
                    waiting_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                
                cv2.putText(waiting_frame, "Waiting for OBS stream...", (50, 240), 
                          cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imshow('OBS Stream', waiting_frame)
            
            # 處理按鍵
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('s') and frame is not None:
                # 保存當前幀
                timestamp = int(time.time())
                filename = f"obs_frame_{timestamp}.jpg"
                cv2.imwrite(filename, frame)
                logger.info(f"已保存幀: {filename}")
            elif key == ord('i'):
                # 顯示統計信息
                stats = obs_capture.get_stats()
                logger.info("=== OBS 串流統計 ===")
                for key, value in stats.items():
                    logger.info(f"  {key}: {value}")
    
    except KeyboardInterrupt:
        logger.info("收到中斷信號")
    
    finally:
        # 清理資源
        obs_capture.stop()
        cv2.destroyAllWindows()
        logger.info("OBS 串流捕獲已停止")

def advanced_obs_capture():
    """高級 OBS 捕獲示例 - 支持多個串流源"""
    
    # 載入配置
    config = load_config()
    
    # 創建串流管理器
    manager = OBSStreamManager()
    
    # 添加配置中的所有串流
    for name, stream_config in config["obs_streams"].items():
        success = manager.add_stream(
            name=name,
            stream_url=stream_config["url"],
            capture_audio=stream_config.get("capture_audio", True)
        )
        if success:
            logger.info(f"已添加串流: {name} -> {stream_config['url']}")
    
    # 啟動所有串流
    manager.start_all_streams()
    
    # 創建顯示窗口
    cv2.namedWindow('OBS Streams', cv2.WINDOW_RESIZABLE)
    
    logger.info("多串流 OBS 捕獲已開始")
    logger.info("按 'q' 退出，按 'r' 開始/停止錄製")
    
    try:
        while True:
            # 獲取第一個可用串流的幀
            frame = None
            for stream_name in config["obs_streams"].keys():
                frame = manager.get_stream_frame(stream_name)
                if frame is not None:
                    break
            
            if frame is not None:
                # 添加串流信息
                stats = manager.get_all_stats()
                y_offset = 30
                for name, stat in stats.items():
                    if stat['is_connected']:
                        info_text = f"{name}: FPS {stat['fps']} | Queue {stat['frame_queue_size']}"
                        cv2.putText(frame, info_text, (10, y_offset), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        y_offset += 25
                
                cv2.imshow('OBS Streams', frame)
                
                # 如果正在錄製，記錄這一幀
                if manager.recording:
                    manager.record_frame(frame)
            
            # 處理按鍵
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                # 切換錄製狀態
                if manager.recording:
                    manager.stop_recording()
                    logger.info("錄製已停止")
                else:
                    timestamp = int(time.time())
                    filename = f"obs_recording_{timestamp}.mp4"
                    if manager.start_recording(filename):
                        logger.info(f"開始錄製到: {filename}")
            elif key == ord('s') and frame is not None:
                # 保存當前幀
                timestamp = int(time.time())
                filename = f"obs_multi_frame_{timestamp}.jpg"
                cv2.imwrite(filename, frame)
                logger.info(f"已保存幀: {filename}")
    
    except KeyboardInterrupt:
        logger.info("收到中斷信號")
    
    finally:
        # 清理資源
        manager.stop_all_streams()
        manager.stop_recording()
        cv2.destroyAllWindows()
        logger.info("多串流 OBS 捕獲已停止")

def quick_test():
    """快速測試 - 檢查是否能連接到 OBS"""
    
    # 測試 URL 列表
    test_urls = [
        "rtmp://localhost:1935/live/stream",
        "rtsp://localhost:8554/stream", 
        "http://localhost:8080/stream.m3u8",
        # 加入你的實際 OBS 串流 URL
    ]
    
    logger.info("開始 OBS 連接測試...")
    
    for url in test_urls:
        logger.info(f"測試連接: {url}")
        
        # 嘗試連接
        cap = cv2.VideoCapture(url)
        
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                logger.info(f"✓ 成功連接: {url}")
                logger.info(f"  分辨率: {frame.shape[1]}x{frame.shape[0]}")
                cap.release()
                return url
            else:
                logger.warning(f"✗ 連接成功但無法讀取幀: {url}")
        else:
            logger.warning(f"✗ 無法連接: {url}")
        
        cap.release()
    
    logger.error("所有測試 URL 都無法連接")
    return None

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        mode = sys.argv[1]
        if mode == "test":
            quick_test()
        elif mode == "advanced":
            advanced_obs_capture()
        else:
            simple_obs_capture()
    else:
        print("OBS 串流捕獲示例")
        print("用法:")
        print("  python obs_example.py          # 簡單示例")
        print("  python obs_example.py test     # 連接測試")
        print("  python obs_example.py advanced # 高級示例")
        print()
        
        # 默認運行簡單示例
        simple_obs_capture()
