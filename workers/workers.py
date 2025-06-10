import queue
import time
import cv2

from core.shared_queue import (
    
    camera_frame_queue, camera_frame_queue_drawn, 
    
    
    
    
    face_result_queue, frame_queue, gemini_prompt_queue,
    gemini_response_queue, id_result_queue, log_queue_camera, log_queue_gemini,
    log_queue_reid, log_queue_stream, log_queue_system, stop_event)

from reid.detector import PersonDetector


# --- RTMP Worker ---
def rtmp_worker():
    i = 0
    MAX_FRAME_IDX = 10000  # 編號循環上限，避免 int 無限增長
    while not stop_event.is_set():
        frame = f"Frame_{i}"
        # 只保留最新 frame，若 queue 滿則先移除
        try:
            frame_queue.get_nowait()
        except Exception:
            pass
        try:
            frame_queue.put_nowait(frame)  # 修正: 要放入 frame
        except Exception:
            pass
        log_queue_stream.put(f"[RTMPWorker] 推送 {frame}")
        time.sleep(1)
        i = (i + 1) % MAX_FRAME_IDX  # 讓編號循環，避免無限增長

# --- Face Detector Worker ---
def face_detector_worker():
    detector = PersonDetector()
    while not stop_event.is_set():
        try:
            frame = camera_frame_queue.get(timeout=1)
            boxes, crops = detector.detect(frame)
            # 畫框在 frame 上
            frame_with_boxes = frame.copy()
            for (x1, y1, x2, y2) in boxes:
                cv2.rectangle(frame_with_boxes, (x1, y1), (x2, y2), (0, 255, 0), 2)
            try:
                face_result_queue.get_nowait()
            except Exception:
                pass
            for crop, box in zip(crops, boxes):
                try:
                    face_result_queue.put_nowait({
                        "frame": frame,
                        "crop": crop,
                        "box": box,
                        "frame_with_boxes": frame_with_boxes
                    })
                except Exception:
                    pass
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
            # id_result_queue 只保留最新
            try:
                id_result_queue.get_nowait()
            except Exception:
                pass
            try:
                id_result_queue.put_nowait({
                    "frame": frame,
                    "box": box,
                    "person_id": person_id,
                    "confidence": score,
                })
            except Exception:
                pass
            log_queue_reid.put(
                f"[ReID] {person_id} (可信度: {score:.2f}, 新身份: {is_new})"
            )
        except queue.Empty:
            continue
        except Exception as e:
            log_queue_reid.put(f"[ReID] 發生錯誤：{e}")

# --- Camera Worker ---
def camera_worker(cam_id=0):
    """優先使用 Insta360，失敗時自動 fallback 回 cv2 攝影機"""
    insta_ok = False
    try:
        from Insta_OpenCV.controller.insta_worker import InstaWorker
        log_queue_system.put("[CameraWorker] 嘗試啟動 Insta360 攝影機...")
        insta_worker = InstaWorker()
        # 第一階段：啟動 ready_event 最多等 30 秒
        ready_event = insta_worker.start_all()
        log_queue_system.put("[CameraWorker] Insta360 啟動中，等待 ready_event (最多 30 秒)...")
        if not ready_event.wait(timeout=30):
            log_queue_system.put("[CameraWorker] Insta360 啟動逾時，將 fallback 至 cv2 攝影機")
        else:
            # 第二階段：啟動成功後，最多再等 10 秒等第一張畫面
            log_queue_system.put("[CameraWorker] Insta360 啟動成功，等待第一張畫面 (最多 10 秒)...")
            frame = None
            for _ in range(100):  # 10 秒，每 0.1 秒檢查一次
                frame = insta_worker.get_latest_frame()
                if frame is not None:
                    break
                time.sleep(0.1)
            if frame is not None:
                # 啟動前清空 camera_frame_queue，避免殘留舊畫面（即使 maxsize=1，也用 while 以防萬一）
                while True:
                    try:
                        camera_frame_queue.get_nowait()
                    except Exception:
                        break
                log_queue_system.put("[CameraWorker] Insta360 取得第一張畫面，進入影像推送模式")
                insta_ok = True

                while not stop_event.is_set():
                    frame = insta_worker.get_latest_frame()
                    if frame is not None:
                        # 只保留最新一張畫面
                        try:
                            camera_frame_queue.get_nowait()
                        except Exception:
                            pass
                        try:
                            camera_frame_queue.put_nowait(frame)
                        except Exception:
                            pass
                    else:
                        log_queue_system.put("[CameraWorker] Insta360 無法取得 frame，等待中...")
                        time.sleep(0.1)
                    time.sleep(0.03)
                insta_worker.stop_all()
                log_queue_system.put("[CameraWorker] Insta360 已關閉")
            else:
                log_queue_system.put("[CameraWorker] Insta360 啟動但 10 秒內無法取得畫面，將 fallback 至 cv2 攝影機")
    except Exception as e:
        log_queue_system.put(f"[CameraWorker] Insta360 啟動失敗，錯誤：{e}，將 fallback 至 cv2 攝影機")

    if not insta_ok:
        # Fallback: 原本的 cv2 攝影機
        cap = cv2.VideoCapture(cam_id)
        if not cap.isOpened():
            log_queue_system.put("[CameraWorker] 無法啟動攝影機，請確認裝置是否連接")
            return
        log_queue_system.put(f"[CameraWorker] 攝影機啟動成功（ID: {cam_id}）")
        # 啟動前先清空 camera_frame_queue，避免殘留舊畫面
        try:
            camera_frame_queue.get_nowait()
        except Exception:
            pass
        while not stop_event.is_set() and cap.isOpened():
            ret, frame = cap.read()
            if ret:
                # 只保留最新一張畫面
                try:
                    camera_frame_queue.get_nowait()
                except Exception:
                    pass
                try:
                    camera_frame_queue.put_nowait(frame)
                except Exception:
                    pass
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