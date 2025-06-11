# controller/insta_worker.py

import threading
from ..controller.insta_controller import InstaController
from ..utils.frame_receiver import FrameReceiver
from ..services.heartbeat import HeartbeatService
import asyncio

class InstaWorker:
    """
    高階 Insta360 控制協調器，負責自動管理 InstaController、心跳、FrameReceiver 等生命週期。
    UI/測試端只需調用 InstaWorker 高階 API。
    """
    def __init__(self):
        self.controller = InstaController()
        self.heartbeat = None
        self.frame_receiver = None
        self._ready_event = threading.Event()
        self._loop = None
        self._thread = None

    def start_all(self, on_stream_error=None):
        """
        啟動所有服務（connect、心跳、RTMP 串流、FrameReceiver），回傳 ready event。
        可選 on_stream_error callback，供 UI 顯示串流異常。
        """
        self._ready_event.clear()
        self._on_stream_error = on_stream_error
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._start_async, daemon=True)
        self._thread.start()
        return self._ready_event

    def _start_async(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._async_start_all())

    async def _async_start_all(self):
        await self.controller.connect()
        self.heartbeat = HeartbeatService(self.controller)
        heartbeat_task = asyncio.create_task(self.heartbeat.run())
        await self.controller.start_preview()
        # --- 確認 RTMP 串流可用再啟動 FrameReceiver ---
        import cv2, time
        stream_url = self.controller.get_stream_url()
        max_wait = 10
        start_time = time.time()
        while True:
            try:
                cap = cv2.VideoCapture(stream_url)
                if cap.isOpened():
                    cap.release()
                    print(f"[Worker] RTMP stream ready: {stream_url}")
                    break
                else:
                    cap.release()
                    if time.time() - start_time > max_wait:
                        print(f"[Worker] RTMP stream not ready after {max_wait}s, aborting.")
                        self._ready_event.set()
                        return
                    print("[Worker] Waiting for RTMP stream to be ready...")
                    time.sleep(1)
            except Exception as e:
                # 僅在 debug 模式下顯示 socket error，否則完全靜音
                if hasattr(self, '_debug') and self._debug:
                    print(f"[Worker] RTMP check exception: {e}")
                time.sleep(1)
        # 傳入 on_stream_error callback
        self.frame_receiver = FrameReceiver(stream_url)
        self.frame_receiver.start(on_error=self._on_stream_error)
        self._ready_event.set()
        await heartbeat_task

    def get_latest_frame(self):
        """
        取得最新 frame，給 UI 或其他模組用。
        若 frame_receiver 尚未啟動或尚未收到畫面，回傳 None。
        """
        if self.frame_receiver:
            return self.frame_receiver.get_latest_frame()
        return None

    def stop_all(self):
        """
        結束所有服務（心跳、FrameReceiver）。
        可依需求擴充：如 self.controller.stop_preview() 釋放相機端資源。
        """
        if self.heartbeat:
            self.heartbeat.stop()
        if self.frame_receiver:
            self.frame_receiver.stop()
        # 可加 self.controller.stop_preview() 等
