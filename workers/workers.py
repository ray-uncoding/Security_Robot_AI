import queue
import random
import time

import cv2

from ai.gemini_client import GeminiClient  # Import the new client
from core.shared_queue import (  # Added gemini log queue; Added gemini queues
    camera_frame_queue, face_result_queue, frame_queue, gemini_prompt_queue,
    gemini_response_queue, id_result_queue, log_queue_camera, log_queue_gemini,
    log_queue_reid, log_queue_stream, log_queue_system, stop_event)
from reid.reid_manager import ReIDManager


def rtmp_worker():
    i = 0
    while not stop_event.is_set():
        frame = f"Frame_{i}"
        frame_queue.put(frame)
        log_queue_stream.put(f"[RTMPWorker] 推送 {frame}")
        time.sleep(1)
        i += 1

def face_detector_worker():
    from reid.detector import PersonDetector
    detector = PersonDetector()

    while not stop_event.is_set():
        try:
            frame = camera_frame_queue.get(timeout=1)
            boxes, crops = detector.detect(frame)

            # 將檢測框與裁剪圖像放入 face_result_queue
            for crop, box in zip(crops, boxes):
                face_result_queue.put({"frame": frame, "crop": crop, "box": box})

            log_queue_camera.put(f"[FaceDetector] 檢測到 {len(boxes)} 個人物")
        except queue.Empty:
            continue
        except Exception as e:
            log_queue_camera.put(f"[FaceDetector] 發生錯誤：{e}")

def reid_worker():
    from reid.reid_manager import ReIDManager
    reid_manager = ReIDManager()

    while not stop_event.is_set():
        try:
            face_data = face_result_queue.get(timeout=1)
            crop = face_data.get("crop")
            frame = face_data.get("frame")
            box = face_data.get("box")

            # 使用 ReIDManager 處理特徵
            feature = reid_manager.reid.extract(crop)
            person_id, score, is_new = reid_manager.db.match_or_register(feature)

            # 將結果放入 id_result_queue，供 UI 顯示
            id_result_queue.put({
                "frame": frame,
                "box": box,
                "person_id": person_id,
                "confidence": score,
            })
            log_queue_reid.put(
                f"[ReID] {person_id} (可信度: {score:.2f}, 新身份: {is_new})"
            )
        except queue.Empty:
            continue
        except Exception as e:
            log_queue_reid.put(f"[ReID] 發生錯誤：{e}")

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