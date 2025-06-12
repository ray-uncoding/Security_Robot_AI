# controller/insta_worker.py

import threading
import asyncio
import time
import cv2
from ..controller.insta_controller import InstaController
from ..utils.frame_receiver import FrameReceiver
from ..services.heartbeat import HeartbeatService

class InstaWorker:
    """
    高階 Insta360 控制協調器，負責自動管理 InstaController、心跳、FrameReceiver 等生命週期。
    UI/測試端只需調用 InstaWorker 高階 API。
    """
    def __init__(self):
        self.controller = InstaController()
        self.heartbeat = None
        self.preview_receiver = None  # 單路 preview
        self.live_receivers = {}      # 多路 live: {i: FrameReceiver}
        self._ready_event = threading.Event()
        self._loop = None
        self._thread = None

    def start_preview_all(self, on_stream_error=None):
        """
        啟動所有服務（connect、心跳、RTMP 預覽串流、FrameReceiver），回傳 ready event。
        """
        self._ready_event.clear()
        self._on_stream_error = on_stream_error
        self._mode = 'preview'
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._start_async, daemon=True)
        self._thread.start()
        return self._ready_event

    def start_live_all(self, on_stream_error=None):
        """
        啟動所有服務（connect、心跳、RTMP live 串流、FrameReceiver），回傳 ready event。
        """
        self._ready_event.clear()
        self._on_stream_error = on_stream_error
        self._mode = 'live'
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._start_async, daemon=True)
        self._thread.start()
        return self._ready_event

    def _start_async(self):
        asyncio.set_event_loop(self._loop)
        if getattr(self, '_mode', None) == 'live':
            self._loop.run_until_complete(self._async_start_live())
        else:
            self._loop.run_until_complete(self._async_start_preview())

    async def _async_start_preview(self):
        await self.controller.connect()
        self.heartbeat = HeartbeatService(self.controller)
        heartbeat_task = asyncio.create_task(self.heartbeat.run())
        await self.controller.start_preview()
        # 不再驗證 RTMP stream，不啟動 FrameReceiver
        self._ready_event.set()
        await heartbeat_task

    async def _async_start_live(self):
        await self.controller.connect()
        self.heartbeat = HeartbeatService(self.controller)
        heartbeat_task = asyncio.create_task(self.heartbeat.run())
        # 確認 nginx 是否啟動，若未啟動則嘗試啟動並等待啟動完成
        import psutil, os, time, subprocess
        nginx_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../nginx 1.7.11.3 Gryphon/nginx.exe'))
        nginx_cwd = os.path.dirname(nginx_path)
        nginx_running = any('nginx.exe' in p.name().lower() for p in psutil.process_iter())
        if not nginx_running:
            try:
                subprocess.Popen([nginx_path], cwd=nginx_cwd, creationflags=subprocess.CREATE_NEW_CONSOLE)
                print('[Worker] nginx.exe 啟動中...')
                # 等待 nginx 啟動成功
                for _ in range(10):
                    time.sleep(1)
                    nginx_running = any('nginx.exe' in p.name().lower() for p in psutil.process_iter())
                    if nginx_running:
                        print('[Worker] nginx.exe 啟動成功')
                        break
                else:
                    print('[Worker] nginx.exe 啟動失敗，請檢查路徑與權限')
            except Exception as e:
                print(f'[Worker] 啟動 nginx 失敗: {e}')
        await self.controller.start_live()
        # 不再驗證 RTMP stream，不啟動 FrameReceiver
        self._ready_event.set()
        await heartbeat_task

    def start_preview_receiver(self, stream_url):
        """手動啟動 preview FrameReceiver。"""
        if self.preview_receiver:
            self.preview_receiver.stop()
        self.preview_receiver = FrameReceiver(stream_url)
        self.preview_receiver.start()

    def get_latest_preview_frame(self):
        """取得最新 preview frame。"""
        if self.preview_receiver:
            return self.preview_receiver.get_latest_frame()
        return None

    def start_live_receiver(self, i, stream_url):
        """手動啟動第 i 路 live FrameReceiver。"""
        if i in self.live_receivers:
            self.live_receivers[i].stop()
        receiver = FrameReceiver(stream_url)
        receiver.start()
        self.live_receivers[i] = receiver

    def get_latest_live_frame(self, i):
        """取得第 i 路 live 最新 frame。"""
        receiver = self.live_receivers.get(i)
        if receiver:
            return receiver.get_latest_frame()
        return None

    def stop_all(self):
        """
        結束所有服務（心跳、FrameReceiver、相機端 preview/live）。
        """
        if self.heartbeat:
            self.heartbeat.stop()
            self.heartbeat = None
        if self.preview_receiver:
            self.preview_receiver.stop()
            self.preview_receiver = None
        for receiver in self.live_receivers.values():
            receiver.stop()
        self.live_receivers.clear()
        try:
            if hasattr(self, '_mode'):
                if self._mode == 'preview':
                    self.controller.stop_preview_sync()
                elif self._mode == 'live':
                    self.controller.stop_live_sync()
        except Exception as e:
            print(f"[Worker] stop_all: stop_preview/stop_live failed: {e}")
