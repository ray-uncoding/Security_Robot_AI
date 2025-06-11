# controller/insta_controller.py

import aiohttp
import json
import os
from ..utils.config_loader import load_settings, save_settings
from ..utils.frame_receiver import FrameReceiver
from ..services.heartbeat import HeartbeatService
import threading
import asyncio

class InstaController:
    """
    Insta360 Pro 相機非同步控制器
    - 初始化、連線、RTMP 串流、拍照、心跳、重連
    - aiohttp + asyncio 架構
    - 僅保留底層 API 操作，不再負責高階協調
    - start_all、stop_all、get_latest_frame 等高階功能已移至 InstaWorker
    """
    def __init__(self):
        self.session: aiohttp.ClientSession | None = None
        self.settings = load_settings()
        http_port = self.settings.get("http_port", 20000)
        self.base_url = f"http://{self.settings['insta_ip']}:{http_port}/osc"
        self.fingerprint = self.settings.get("fingerprint", "")
        # 讀取 API payload 設定
        payload_path = os.path.join(os.path.dirname(__file__), "../config/api_payloads.json")
        with open(payload_path, "r", encoding="utf-8") as f:
            self.api_payloads = json.load(f)

    def _get_headers(self):
        return {
            "Content-Type": "application/json",
            "Fingerprint": self.fingerprint
        }

    async def connect(self):
        """建立與相機的會話連線，取得 fingerprint"""
        import datetime
        url = f"{self.base_url}/commands/execute"
        now = datetime.datetime.now(datetime.timezone.utc)
        hw_time = now.strftime("%m%d%H%M%Y.%S")
        payload = json.loads(json.dumps(self.api_payloads["connect"]))
        payload["parameters"]["hw_time"] = hw_time
        async with aiohttp.ClientSession() as session:
            async with session.post(url, json=payload, headers={"Content-Type": "application/json"}) as resp:
                try:
                    data = await resp.json(content_type=None)
                except Exception as e:
                    text = await resp.text()
                    try:
                        text_json = json.loads(text)
                        text = json.dumps(text_json, indent=2, ensure_ascii=False)
                    except Exception:
                        pass
                    print(f"[connect] 無法解析 JSON，HTTP狀態: {resp.status}, 內容如下:\n{text}")
                    raise RuntimeError(f"connect 失敗: {e}, HTTP狀態: {resp.status}, 內容: {text}")
                self.fingerprint = data.get("results", {}).get("Fingerprint", "")
                self.settings["fingerprint"] = self.fingerprint
                save_settings(self.settings)

    async def start_preview(self):
        """啟動 RTMP 串流預覽"""
        if not self.fingerprint:
            raise RuntimeError("尚未取得 fingerprint，請先 connect()")
        payload = self.api_payloads["start_preview"]
        cmd_url = f"{self.base_url}/commands/execute"
        async with aiohttp.ClientSession() as session:
            async with session.post(cmd_url, json=payload, headers=self._get_headers()) as resp2:
                try:
                    resp_data2 = await resp2.json(content_type=None)
                except Exception as e:
                    text = await resp2.text()
                    try:
                        text_json = json.loads(text)
                        text = json.dumps(text_json, indent=2, ensure_ascii=False)
                    except Exception:
                        pass
                    print(f"[start_preview] 無法解析 JSON，HTTP狀態: {resp2.status}, 內容如下:\n{text}")
                    raise RuntimeError(f"start_preview 失敗: {e}, HTTP狀態: {resp2.status}, 內容: {text}")
                if resp_data2.get("state") != "done":
                    print(f"start_preview 回傳異常: {json.dumps(resp_data2, indent=2, ensure_ascii=False)}")

    async def stop_preview(self):
        """停止 RTMP 串流"""
        if not self.fingerprint:
            return
        payload = self.api_payloads["stop_preview"]
        url = f"{self.base_url}/commands/execute"
        async with aiohttp.ClientSession() as session:
            async with session.post(url, json=payload, headers=self._get_headers()) as resp:
                data = await resp.json(content_type=None)
                if data.get("state") != "done":
                    raise RuntimeError(f"stopPreview 失敗: {data}")

    async def take_picture(self):
        """拍攝靜態圖片，回傳 sequence"""
        if not self.fingerprint:
            return None
        payload = self.api_payloads["take_picture"]
        url = f"{self.base_url}/commands/execute"
        async with aiohttp.ClientSession() as session:
            async with session.post(url, json=payload, headers=self._get_headers()) as resp:
                data = await resp.json(content_type=None)
                sequence = data.get("sequence")
                if not sequence:
                    raise RuntimeError(f"takePicture 無法取得 sequence: {data}")
                return sequence

    async def get_async_result(self, sequence):
        """查詢 async 指令結果"""
        payload = json.loads(json.dumps(self.api_payloads["get_result"]))
        payload["parameters"]["list_ids"] = [sequence]
        url = f"{self.base_url}/commands/execute"
        import asyncio
        for _ in range(20):
            async with aiohttp.ClientSession() as session:
                async with session.post(url, json=payload, headers=self._get_headers()) as resp:
                    data = await resp.json(content_type=None)
                    results = data.get("results", {})
                    if results.get("state") == "done":
                        return results
                    elif results.get("state") == "error":
                        raise RuntimeError(f"async 指令失敗: {results}")
            await asyncio.sleep(1)
        raise TimeoutError("async 指令查詢逾時")

    async def send_heartbeat(self):
        """維持連線活性 (每秒呼叫一次 /osc/state)"""
        if not self.fingerprint:
            return
        url = f"{self.base_url}/state"
        async with aiohttp.ClientSession() as session:
            async with session.post(url, json={}, headers=self._get_headers()) as resp:
                if resp.status != 200:
                    raise ConnectionError("❌ 心跳失敗，連線異常")

    async def reconnect(self):
        """心跳失敗時自動重連 (connect → start_preview)"""
        print("🔄 嘗試重新連線 Insta360 相機...")
        await self.connect()
        await self.start_preview()

    def get_stream_url(self) -> str:
        """動態組合 RTMP 預覽串流網址 (官方格式)"""
        ip = self.settings.get("insta_ip", "127.0.0.1")
        return f"rtmp://{ip}:1935/live/preview"
