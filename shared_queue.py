import queue
import threading

# 全域佇列（模組間共享）
frame_queue = queue.Queue()
face_result_queue = queue.Queue()
id_result_queue = queue.Queue()

# UI 日誌佇列
log_queue_camera = queue.Queue()
log_queue_stream = queue.Queue()
log_queue_reid = queue.Queue()
log_queue_system = queue.Queue()
log_queue_gemini = queue.Queue() # New log queue for Gemini

# 控制旗標
stop_event = threading.Event()

# 相機畫面 frame queue
camera_frame_queue = queue.Queue()

# Gemini 溝通佇列
gemini_prompt_queue = queue.Queue() # UI -> Worker (prompt, model_name)
gemini_response_queue = queue.Queue() # Worker -> UI (response_text)