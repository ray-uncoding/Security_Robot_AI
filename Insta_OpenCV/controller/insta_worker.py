# controller/insta_worker.py

import threading
import asyncio
import time
from ..controller.insta_controller import InstaController
from ..services.heartbeat import HeartbeatService

class InstaWorker:
    """
    高階 Insta360 控制協調器，負責自動管理 InstaController、心跳等生命週期。
    UI/測試端只需調用 InstaWorker 高階 API。
    """
    def __init__(self, ip_address="192.168.1.1"):
        # When creating the controller, pass the IP so it knows where to connect
        self.controller = InstaController(ip_address=ip_address)
        self.heartbeat = None
        self._ready_event = threading.Event()
        self._loop = None
        self._thread = None

    def start_preview_all(self, on_stream_error=None):
        """
        啟動所有服務（connect、心跳、RTMP 預覽串流），回傳 ready event。
        """
        self._ready_event.clear()
        self._mode = 'preview'
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._start_async, daemon=True)
        self._thread.start()
        return self._ready_event

    def start_live_all(self, on_stream_error=None):
        """
        啟動所有服務（connect、心跳、RTMP live 串流），回傳 ready event。
        """
        self._ready_event.clear()
        self._mode = 'live'
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._start_async, daemon=True)
        self._thread.start()
        return self._ready_event

    def _start_async(self):
        asyncio.set_event_loop(self._loop)
        # Connect first, then decide which stream to start
        self._loop.run_until_complete(self._async_connect_and_start())

    async def _async_connect_and_start(self):
        """Helper to connect, start heartbeat, and then start the selected stream."""
        try:
            # Connect to the camera and initialize the session
            await self.controller.connect()
            
            # Start the heartbeat to keep the connection alive
            self.heartbeat = HeartbeatService(self.controller)
            heartbeat_task = asyncio.create_task(self.heartbeat.run())

            # Start the appropriate stream based on the mode
            if getattr(self, '_mode', None) == 'live':
                await self.controller.start_live()
            else:
                await self.controller.start_preview()
            
            # Signal that the worker is ready
            self._ready_event.set()
            
            # Keep the heartbeat running
            await heartbeat_task
        except Exception as e:
            print(f"[Worker] Critical error in async startup: {e}")
            self._ready_event.set() # Signal ready anyway to prevent blocking, but with an error

    def stop_all(self):
        """
        結束所有服務（心跳、相機端 preview/live）。
        """
        if self.heartbeat:
            self.heartbeat.stop()
            self.heartbeat = None
        try:
            if hasattr(self, '_mode'):
                if self._mode == 'preview':
                    self.controller.stop_preview_sync()
                elif self._mode == 'live':
                    self.controller.stop_live_sync()
        except Exception as e:
            print(f"[Worker] stop_all: stop_preview/stop_live failed: {e}")

