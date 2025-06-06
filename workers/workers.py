import queue
import time
import cv2

from core.shared_queue import (
    camera_frame_queue, face_result_queue, frame_queue, gemini_prompt_queue,
    gemini_response_queue, id_result_queue, log_queue_camera, log_queue_gemini,
    log_queue_reid, log_queue_stream, log_queue_system, stop_event)

# --- RTMP Worker ---
def rtmp_worker():
    i = 0
    while not stop_event.is_set():
        frame = f"Frame_{i}"
        frame_queue.put(frame)
        log_queue_stream.put(f"[RTMPWorker] 推送 {frame}")
        time.sleep(1)
        i += 1

# --- Face Detector Worker ---
def face_detector_worker():
    from reid.detector import PersonDetector
    detector = PersonDetector()
    while not stop_event.is_set():
        try:
            frame = camera_frame_queue.get(timeout=1)
            boxes, crops = detector.detect(frame)
            for crop, box in zip(crops, boxes):
                face_result_queue.put({"frame": frame, "crop": crop, "box": box})
            log_queue_camera.put(f"[FaceDetector] 檢測到 {len(boxes)} 個人物")
        except queue.Empty:
            continue
        except Exception as e:
            log_queue_camera.put(f"[FaceDetector] 發生錯誤：{e}")

# --- ReID Worker ---
def reid_worker():
    from reid.reid_manager import ReIDManager
    reid_manager = ReIDManager()
    while not stop_event.is_set():
        try:
            face_data = face_result_queue.get(timeout=1)
            crop = face_data.get("crop")
            frame = face_data.get("frame")
            box = face_data.get("box")
            feature = reid_manager.reid.extract(crop)
            person_id, score, is_new = reid_manager.db.match_or_register(feature)
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

# --- Camera Worker ---
def camera_worker(cam_id=0):
    cap = cv2.VideoCapture(cam_id)
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
        time.sleep(0.03)
    cap.release()
    log_queue_system.put("[CameraWorker] 攝影機已關閉")

# --- Gemini Worker ---
def gemini_worker():
    from AI_api import create_client, text_chat_async
    log_queue_gemini.put("[GeminiWorker] Initializing...")
    client = create_client()
    current_model = None
    import asyncio
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    while not stop_event.is_set():
        try:
            prompt_data = gemini_prompt_queue.get(timeout=1)
            prompt_text = prompt_data.get("prompt")
            model_name = prompt_data.get("model")
            if not prompt_text or not model_name:
                log_queue_gemini.put("[GeminiWorker] Invalid prompt data received.")
                continue
            if model_name != current_model:
                if client.set_model(model_name):
                    current_model = model_name
                else:
                    gemini_response_queue.put(f"Error: Failed to set Gemini model to {model_name}")
                    continue
            try:
                response = loop.run_until_complete(text_chat_async(client, prompt_text))
                gemini_response_queue.put(response)
            except Exception as e:
                log_queue_gemini.put(f"[GeminiWorker] Error in async chat: {e}")
                gemini_response_queue.put(f"Error: Gemini async chat failed. {e}")
        except queue.Empty:
            continue
        except Exception as e:
            log_queue_gemini.put(f"[GeminiWorker] Error processing prompt: {e}")
            gemini_response_queue.put(f"Error: An internal error occurred in Gemini worker. {e}")
    log_queue_gemini.put("[GeminiWorker] Shutting down.")