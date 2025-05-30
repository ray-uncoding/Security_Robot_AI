"""Global queues and events for inter-thread communication.

This module defines various queues used for passing data between different
components of the Security Robot AI system (e.g., camera frames,
detection results, log messages, Gemini prompts/responses). It also
defines a global event for signaling threads to stop.
"""

import queue
import threading
from typing import Any

# Global queues for data exchange between modules
frame_queue: queue.Queue[Any] = queue.Queue()
face_result_queue: queue.Queue[Any] = queue.Queue()
id_result_queue: queue.Queue[Any] = queue.Queue()

# UI Log Queues
log_queue_camera: queue.Queue[str] = queue.Queue()
log_queue_stream: queue.Queue[str] = queue.Queue()
log_queue_reid: queue.Queue[str] = queue.Queue()
log_queue_system: queue.Queue[str] = queue.Queue()
log_queue_gemini: queue.Queue[str] = queue.Queue()  # Log queue for Gemini client

# Control Flag
stop_event: threading.Event = threading.Event()

# Camera frame queue (specific for raw camera frames if different from frame_queue)
camera_frame_queue: queue.Queue[Any] = queue.Queue()

# Gemini Communication Queues
gemini_prompt_queue: queue.Queue[Any] = queue.Queue()  # UI -> Worker (prompt, model_name)
gemini_response_queue: queue.Queue[Any] = queue.Queue()  # Worker -> UI (response_text)