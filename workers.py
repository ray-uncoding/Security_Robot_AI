"""Worker threads for various system components.

This module contains worker functions that run in separate threads to handle
camera capture, stream processing, re-identification, and Gemini AI interactions.
"""

import queue
import random
import time
from typing import Optional

import cv2

from gemini_client import GeminiClient  # Import the new client
from shared_queue import (  # Added gemini log queue; Added gemini queues
    camera_frame_queue, face_result_queue, frame_queue, gemini_prompt_queue,
    gemini_response_queue, id_result_queue, log_queue_camera, log_queue_gemini,
    log_queue_reid, log_queue_stream, log_queue_system, stop_event)


def rtmp_worker():
    i = 0
    while not stop_event.is_set():
        frame = f"Frame_{i}"
        frame_queue.put(frame)
        log_queue_stream.put(f"[RTMPWorker] 推送 {frame}")
        time.sleep(1)
        i += 1

def face_detector_worker():
    while not stop_event.is_set():
        try:
            frame = frame_queue.get(timeout=1)
            face_features = {
                "frame": frame,
                "face_id": f"face_{random.randint(100,999)}",
                "embedding": [random.random() for _ in range(128)]
            }
            face_result_queue.put(face_features)
            log_queue_camera.put(f"[FaceDetector] 偵測出 {face_features['face_id']} from {frame}")
        except queue.Empty:
            continue

def reid_worker():
    while not stop_event.is_set():
        try:
            face_data = face_result_queue.get(timeout=1)
            person_id = f"Person_{random.randint(1, 5)}"
            result = {
                "frame": face_data["frame"],
                "face_id": face_data["face_id"],
                "person_id": person_id,
                "confidence": round(random.uniform(0.85, 0.99), 2)
            }
            id_result_queue.put(result)
            log_queue_reid.put(
                f"[ReID] {result['face_id']} ➔ {result['person_id']} (可信度: {result['confidence']})"
            )
        except queue.Empty:
            continue

def camera_worker(cam_id=0):
    cap = cv2.VideoCapture(cam_id)  # 預設為 USB 攝影機 0
    #rtmp_url = "rtmp://live.hkstv.hk.lxdns.com/live/hks"  # 香港直播頻道
    #cap = cv2.VideoCapture(rtmp_url)  # 預設為 USB 攝影機 0
    
    if not cap.isOpened():
        log_queue_system.put("[CameraWorker] 無法啟動攝影機，請確認裝置是否連接")
        return
    
    log_queue_system.put(f"[CameraWorker] 攝影機啟動成功（ID: {cam_id}）")
    
    while not stop_event.is_set() and cap.isOpened():
        ret, frame = cap.read()
        if ret:
            if not camera_frame_queue.full():
                camera_frame_queue.put(frame)
        else:
            break
        time.sleep(0.03)  # 每秒約 30 幀
    
    cap.release()
    log_queue_system.put("[CameraWorker] 攝影機已關閉")

# --- New Gemini Worker ---
def gemini_worker() -> None:
    """Worker thread for handling Gemini API interactions.
    
    Reads prompts from gemini_prompt_queue and sends responses to gemini_response_queue.
    Implements quota error handling with cooldown period.
    """
    log_queue_gemini.put("[GeminiWorker] Initializing...")
    client = GeminiClient()
    current_model: Optional[str] = None
    
    # Track quota error timing to avoid frequent retries
    last_quota_error_time: float = 0
    quota_error_cooldown: int = 60  # Wait 60 seconds after quota error

    while not stop_event.is_set():
        try:
            # Get prompt and desired model from the queue
            prompt_data = gemini_prompt_queue.get(timeout=1)
            prompt_text = prompt_data.get("prompt")
            model_name = prompt_data.get("model")

            if not prompt_text or not model_name:
                log_queue_gemini.put("[GeminiWorker] Invalid prompt data received.")
                continue

            # 檢查是否在配額錯誤冷卻期間
            current_time = time.time()
            if last_quota_error_time > 0 and (current_time - last_quota_error_time) < quota_error_cooldown:
                remaining_cooldown = int(quota_error_cooldown - (current_time - last_quota_error_time))
                error_msg = f"配額錯誤: API 配額已用盡。請等待 {remaining_cooldown} 秒後再試。\n\n建議使用 check_api_quota.py 工具檢查配額狀態。"
                gemini_response_queue.put(error_msg)
                log_queue_gemini.put(f"[GeminiWorker] 配額錯誤冷卻中，剩餘 {remaining_cooldown} 秒")
                continue

            # Update model if changed
            if model_name != current_model:
                if client.set_model(model_name):
                    current_model = model_name
                    log_queue_gemini.put(f"[GeminiWorker] 模型已切換至: {model_name}")
                else:
                    # Failed to set model
                    gemini_response_queue.put(f"錯誤: 無法設定 Gemini 模型為 {model_name}")
                    continue  # Skip generation if model setup failed

            # Generate response
            log_queue_gemini.put(f"[GeminiWorker] 正在生成回應...")
            response = client.generate_response(prompt_text)
            
            # 檢查回應是否為配額錯誤
            if "配額錯誤:" in response:
                last_quota_error_time = current_time
                log_queue_gemini.put("[GeminiWorker] 檢測到配額超限錯誤")
            else:
                # 成功的回應，重置配額錯誤時間
                last_quota_error_time = 0
                
            gemini_response_queue.put(response)
            log_queue_gemini.put(f"[GeminiWorker] 回應已生成並發送到 UI")

        except queue.Empty:
            # No prompt received, continue loop
            continue
        except Exception as e:
            error_str = str(e)
            log_queue_gemini.put(f"[GeminiWorker] 處理提示時發生錯誤: {error_str}")
            
            # 檢查是否為配額相關錯誤
            if any(indicator in error_str.lower() for indicator in ["quota", "rate limit", "1011", "429"]):
                last_quota_error_time = time.time()
                error_msg = "配額錯誤: API 配額已用盡。\n\n請採取以下措施：\n1. 等待配額重置（通常為每分鐘或每日限制）\n2. 檢查 Google AI Studio 中的配額使用情況\n3. 考慮升級到付費方案以獲得更高配額\n4. 使用 check_api_quota.py 工具診斷問題"
                gemini_response_queue.put(error_msg)
            else:
                gemini_response_queue.put(f"錯誤: Gemini worker 內部錯誤。{error_str}")

    log_queue_gemini.put("[GeminiWorker] Shutting down.")