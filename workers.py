import queue
import random
import time

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
def gemini_worker():
    """Handles Gemini API requests from the UI."""
    log_queue_gemini.put("[GeminiWorker] Initializing...")
    client = GeminiClient()
    current_model = None

    while not stop_event.is_set():
        try:
            # Get prompt and desired model from the queue
            prompt_data = gemini_prompt_queue.get(timeout=1)
            prompt_text = prompt_data.get("prompt")
            model_name = prompt_data.get("model")

            if not prompt_text or not model_name:
                log_queue_gemini.put("[GeminiWorker] Invalid prompt data received.")
                continue

            # Update model if changed
            if model_name != current_model:
                if client.set_model(model_name):
                    current_model = model_name
                else:
                    # Failed to set model, put error back?
                    gemini_response_queue.put(f"Error: Failed to set Gemini model to {model_name}")
                    continue # Skip generation if model setup failed

            # Generate response
            if client.model: # Check if model was successfully initialized
                 response = client.generate_response(prompt_text)
                 gemini_response_queue.put(response)
            else:
                 gemini_response_queue.put("Error: Gemini model not available.")


        except queue.Empty:
            # No prompt received, continue loop
            continue
        except Exception as e:
            log_queue_gemini.put(f"[GeminiWorker] Error processing prompt: {e}")
            gemini_response_queue.put(f"Error: An internal error occurred in Gemini worker. {e}")

    log_queue_gemini.put("[GeminiWorker] Shutting down.")