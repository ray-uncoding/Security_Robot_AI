# controller/insta_controller.py

import os
import sys
import json
import asyncio
import threading
import subprocess
import aiohttp
import datetime
import psutil
import nest_asyncio  # 新增，支援巢狀事件循環
from ..utils.config_loader import load_settings, save_settings
from ..utils.frame_receiver import FrameReceiver
from ..services.heartbeat import HeartbeatService

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

    async def start_live(self):
        """啟動 origin live 六路魚眼 RTMP 串流，啟動前自動開啟 nginx"""
        # 啟動 nginx（若尚未啟動）
        nginx_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../nginx 1.7.11.3 Gryphon/nginx.exe'))
        try:
            # Windows: 只啟動一次，不重複開啟
            nginx_running = any('nginx.exe' in p.name().lower() for p in psutil.process_iter())
            if not nginx_running:
                subprocess.Popen([nginx_path], creationflags=subprocess.CREATE_NEW_CONSOLE)
        except Exception as e:
            print(f"[start_live] 啟動 nginx 失敗: {e}")
        if not self.fingerprint:
            raise RuntimeError("尚未取得 fingerprint，請先 connect()")
        payload = self.api_payloads["start_live"]
        cmd_url = f"{self.base_url}/commands/execute"
        async with aiohttp.ClientSession() as session:
            async with session.post(cmd_url, json=payload, headers=self._get_headers()) as resp:
                try:
                    resp_data = await resp.json(content_type=None)
                except Exception as e:
                    text = await resp.text()
                    try:
                        text_json = json.loads(text)
                        text = json.dumps(text_json, indent=2, ensure_ascii=False)
                    except Exception:
                        pass
                    print(f"[start_live] 無法解析 JSON，HTTP狀態: {resp.status}, 內容如下:\n{text}")
                    raise RuntimeError(f"start_live 失敗: {e}, HTTP狀態: {resp.status}, 內容: {text}")
                if resp_data.get("state") != "done":
                    print(f"start_live 回傳異常: {json.dumps(resp_data, indent=2, ensure_ascii=False)}")

    def stop_live_sync(self):
        """同步呼叫 stop_live (for worker, 支援已存在事件循環)"""
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            loop = None
        if loop and loop.is_running():
            nest_asyncio.apply()
            fut = asyncio.ensure_future(self.stop_live())
            loop.run_until_complete(fut)
        else:
            asyncio.run(self.stop_live())

    def stop_preview_sync(self):
        """同步呼叫 stop_preview (for worker, 支援已存在事件循環)"""
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            loop = None
        if loop and loop.is_running():
            nest_asyncio.apply()
            fut = asyncio.ensure_future(self.stop_preview())
            loop.run_until_complete(fut)
        else:
            asyncio.run(self.stop_preview())

    async def stop_live(self):
        """停止 origin live 串流，並嘗試關閉 nginx 服務（僅 Windows）"""
        if not self.fingerprint:
            return
        payload = self.api_payloads["stop_live"]
        url = f"{self.base_url}/commands/execute"
        async with aiohttp.ClientSession() as session:
            async with session.post(url, json=payload, headers=self._get_headers()) as resp:
                data = await resp.json(content_type=None)
                if data.get("state") != "done":
                    raise RuntimeError(f"stop_live 失敗: {data}")
        # 關閉 nginx
        try:
            for p in psutil.process_iter():
                if 'nginx.exe' in p.name().lower():
                    p.terminate()
                    try:
                        p.wait(timeout=3)
                    except Exception:
                        p.kill()
                    print("[stop_live] nginx.exe 已關閉")
        except Exception as e:
            print(f"[stop_live] 關閉 nginx 失敗: {e}")
