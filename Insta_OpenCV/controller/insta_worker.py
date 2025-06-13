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
    def __init__(self):
        self.controller = InstaController()
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
        if getattr(self, '_mode', None) == 'live':
            self._loop.run_until_complete(self._async_start_live())
        else:
            self._loop.run_until_complete(self._async_start_preview())

    async def _async_start_preview(self):
        await self.controller.connect()
        self.heartbeat = HeartbeatService(self.controller)
        heartbeat_task = asyncio.create_task(self.heartbeat.run())
        await self.controller.start_preview()
        # 不再驗證 RTMP stream
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
        # 不再驗證 RTMP stream
        self._ready_event.set()
        await heartbeat_task

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
