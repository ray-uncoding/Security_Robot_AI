import threading

from core.shared_queue import stop_event
from workers.workers import (camera_worker,  # Added gemini_worker
                     face_detector_worker, gemini_worker, reid_worker,
                     rtmp_worker)


threads = []

def start_all_threads():
    global threads
    # Check if threads are already running to prevent duplicates if called again
    if any(t.is_alive() for t in threads):
        print("[Core] Threads already running.")
        return

    stop_event.clear() # Ensure stop event is clear before starting

    threads = [
        threading.Thread(target=rtmp_worker, daemon=True, name="worker_rtmp"),
        threading.Thread(target=face_detector_worker, daemon=True, name="worker_face"),
        threading.Thread(target=reid_worker, daemon=True, name="worker_reid"),
        threading.Thread(target=camera_worker, daemon=True, name="worker_camera"),
        threading.Thread(target=gemini_worker, daemon=True, name="worker_gemini"), # Gemini worker 直接啟動
    ]
    print("[Core] Starting all threads...")
    for t in threads:
        t.start()
    print("[Core] All threads started.")

def stop_all_threads():
    """Sets the stop event to signal threads to terminate."""
    print("[Core] Setting stop event...")
    stop_event.set()
    threads.clear() # Clear the list after stopping
