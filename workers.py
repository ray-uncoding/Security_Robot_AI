import time
import random
import queue
from shared_queue import (
    frame_queue, face_result_queue, id_result_queue,
    log_queue_camera, log_queue_stream, log_queue_reid, log_queue_system,
    camera_frame_queue,
    stop_event    
)
import cv2
from shared_queue import camera_frame_queue

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