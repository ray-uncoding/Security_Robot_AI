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

# 控制旗標
stop_event = threading.Event()
