# services/heartbeat.py

import asyncio
# from src.insta360cam.controller.insta_worker import InstaWorker

class HeartbeatService:
    """
    Insta360 相機狀態輪詢與心跳維持服務。

    根據官方 API 建議，應每秒呼叫一次 `/osc/state`，
    以保持 `fingerprint` 所建立的 session 不會被自動關閉。
    本模組提供 async 背景執行的 run() 方法，可用 asyncio.create_task() 啟動。
    """

    def __init__(self, controller, interval: float = 1.0):
        """
        初始化心跳服務。

        Args:
            controller (InstaController): 控制 Insta360 的實例。
            interval (float): 輪詢頻率（秒），預設為 1 秒一次。
        """
        # 避免循環匯入，僅於此處型別檢查時註解
        # from controller.insta_controller import InstaController
        self.controller = controller
        self.interval = interval
        self.running = False

    async def run(self):
        """
        非同步執行主心跳迴圈。
        若偵測到連線異常，將自動觸發 controller.reconnect() 嘗試修復連線。
        """
        self.running = True
        while self.running:
            try:
                await self.controller.send_heartbeat()
                # 可加 log：print("✅ Heartbeat success")
            except Exception as e:
                print(f"⚠️ Heartbeat failed: {e}")
                await self.controller.reconnect()
            await asyncio.sleep(self.interval)

    def stop(self):
        """
        結束心跳服務（設定 running 為 False，結束 loop）。
        """
        self.running = False

# 若真的需要用到 InstaWorker，請在 function 內部再 import
