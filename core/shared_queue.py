import queue
import threading
import time

# 全域佇列（模組間共享）
frame_queue = queue.Queue(maxsize=20)
face_result_queue = queue.Queue(maxsize=20)

# UI 日誌佇列
log_queue_camera = queue.Queue(maxsize=20)
log_queue_stream = queue.Queue(maxsize=20)
log_queue_reid = queue.Queue(maxsize=20)
log_queue_system = queue.Queue(maxsize=20)
log_queue_gemini = queue.Queue(maxsize=20) # New log queue for Gemini

# 紀錄id
id_result_queue = queue.Queue(maxsize=1)

# 控制旗標
stop_event = threading.Event()

# 相機畫面 frame queue（只保留最新一張）
camera_frame_queue = queue.Queue(maxsize=1)

# 新增：有框畫面 queue（只保留最新一張）
camera_frame_queue_drawn = queue.Queue(maxsize=1)

# Gemini 溝通佇列
gemini_prompt_queue = queue.Queue(maxsize=20) # UI -> Worker (prompt, model_name)
gemini_response_queue = queue.Queue(maxsize=20) # Worker -> UI (response_text)

def print_queue_status():
    while True:
        print(
            f"[QueueMonitor] camera_frame_queue: {camera_frame_queue.qsize()}, "
            f"camera_frame_queue_drawn: {camera_frame_queue_drawn.qsize()}, "
            f"frame_queue: {frame_queue.qsize()}, "
            f"face_result_queue: {face_result_queue.qsize()}, "
            f"id_result_queue: {id_result_queue.qsize()}"
        )
        time.sleep(5)

# 只保留最新20筆，超過就刪除舊的
def clear_old_logs():
    for q in [log_queue_camera, log_queue_stream, log_queue_reid, log_queue_system, log_queue_gemini]:
        while q.qsize() > 20:
            try:
                q.get_nowait()
            except Exception:
                break

# 啟動 queue 監控執行緒（模組 import 時自動啟動，若不想自動可移到 main）
threading.Thread(target=print_queue_status, daemon=True).start()
threading.Thread(target=lambda: (time.sleep(5), clear_old_logs()), daemon=True).start()
