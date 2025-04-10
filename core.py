import threading
from workers import rtmp_worker, face_detector_worker, reid_worker, camera_worker
from shared_queue import stop_event

threads = []

def start_all_threads():
    global threads
    threads = [
        threading.Thread(target=rtmp_worker, daemon=True),
        threading.Thread(target=face_detector_worker, daemon=True),
        threading.Thread(target=reid_worker, daemon=True),
        threading.Thread(target=camera_worker, daemon=True),
    ]
    for t in threads:
        t.start()
