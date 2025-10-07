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
import platform # To check OS
import time # Import the time module
import logging
import httpx
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
    def __init__(self, ip_address="192.168.1.1"):
        self.ip_address = ip_address  # Store the IP address
        self.session: httpx.AsyncClient | None = None
        self.settings = load_settings()
        http_port = self.settings.get("http_port", 20000)
        self.base_url = f"http://{ip_address}:{http_port}/osc"
        self.fingerprint = self.settings.get("fingerprint", "")
        # 讀取 API payload 設定
        payload_path = os.path.join(os.path.dirname(__file__), "../config/api_payloads.json")
        with open(payload_path, "r", encoding="utf-8") as f:
            self.api_payloads = json.load(f)

    def _manage_nginx(self, start=True):
        """
        Manages the Nginx server by stopping any existing instance 
        and starting a new one with the project's configuration.
        """
        os_type = platform.system()
        nginx_process_name = "nginx.exe" if os_type == "Windows" else "nginx"
        # Corrected the relative path from 3 levels up to 2 levels up.
        conf_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../nginx 1.7.11.3 Gryphon/conf/nginx.conf'))

        def stop_any_nginx():
            print("[_manage_nginx] Attempting to stop any running Nginx instance...")
            try:
                if os_type == "Windows":
                    # For the bundled windows version, taskkill is effective
                    subprocess.run(['taskkill', '/F', '/IM', 'nginx.exe'], check=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                else: # Linux/macOS
                    # Use 'nginx -s quit' for graceful shutdown with sudo and auto password
                    process = subprocess.Popen(['sudo', '-S', '/usr/sbin/nginx', '-s', 'quit'], 
                                             stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    process.communicate(input=b'nvidia\n')
                time.sleep(1) # Give it a moment to shut down.
                print("[_manage_nginx] Sent stop signal to any running Nginx.")
            except Exception as e:
                print(f"[_manage_nginx] Error while trying to stop Nginx (this might be okay): {e}")

        if start:
            # Always stop any running Nginx first to ensure we use our config.
            stop_any_nginx()
            
            print(f"[_manage_nginx] Starting Nginx with project config on {os_type}...")
            try:
                if os_type == "Windows":
                    nginx_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../nginx 1.7.11.3 Gryphon/nginx.exe'))
                    nginx_cwd = os.path.dirname(nginx_path)
                    subprocess.Popen([nginx_path], cwd=nginx_cwd, creationflags=subprocess.CREATE_NEW_CONSOLE)
                else: # Linux/macOS
                    # Start nginx with sudo and auto password input
                    process = subprocess.Popen(['sudo', '-S', '/usr/sbin/nginx'], 
                                             stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    process.communicate(input=b'nvidia\n')
                time.sleep(2) # Wait for Nginx to initialize.
                is_running = any(nginx_process_name in p.name().lower() for p in psutil.process_iter(['name']))
                if is_running:
                    print("[_manage_nginx] Nginx started successfully with project config.")
                else:
                    print("[_manage_nginx] CRITICAL: Failed to start Nginx with project config.")
            except Exception as e:
                print(f"[_manage_nginx] CRITICAL: Failed to start Nginx: {e}")
        
        else: # Stop our instance
            print(f"[_manage_nginx] Stopping Nginx instance (started with project config)...")
            try:
                # The most reliable way to stop the instance we started is with the same config file.
             if os_type == "Windows":
                 subprocess.run(['taskkill', '/F', '/IM', 'nginx.exe'], check=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
             else:
                process = subprocess.Popen(['sudo', '-S', '/usr/sbin/nginx', '-s', 'quit'], 
                                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                process.communicate(input=b'nvidia\n')
                
                print("[_manage_nginx] Nginx stop signal sent.")
            except Exception as e:
                print(f"[_manage_nginx] Failed to send stop signal to Nginx: {e}")

    def _get_headers(self):
        return {
            "Content-Type": "application/json",
            "Fingerprint": self.fingerprint
        }

    async def connect(self):
        """建立與相機的會話連線，取得 fingerprint"""
        # Create a persistent httpx client session
        self.session = httpx.AsyncClient(timeout=15.0)
        
        url = f"{self.base_url}/commands/execute"
        now = datetime.datetime.now(datetime.timezone.utc)
        hw_time = now.strftime("%m%d%H%M%Y.%S")
        payload = json.loads(json.dumps(self.api_payloads["connect"]))
        payload["parameters"]["hw_time"] = hw_time
        
        try:
            response = await self.session.post(url, json=payload, headers={"Content-Type": "application/json"})
            response.raise_for_status()
            data = response.json()
            
            if data.get("state") == "done":
                self.fingerprint = data.get("results", {}).get("Fingerprint", "")
                if self.fingerprint:
                    self.settings["fingerprint"] = self.fingerprint
                    save_settings(self.settings)
                    logging.info(f"[connect] Successfully connected. Fingerprint: {self.fingerprint}")
                else:
                    raise RuntimeError(f"connect 成功但無法取得 fingerprint: {data}")
            else:
                raise RuntimeError(f"connect 失敗: {data}")
                
        except Exception as e:
            if self.session:
                await self.session.aclose()
                self.session = None
            raise RuntimeError(f"connect 失敗: {e}")

    async def start_preview(self, stream_type='rtmp'):
        """
        Starts the camera preview using the correct OSC command format.
        :param stream_type: 'rtmp' or 'low_latency'
        """
        print(f"[start_preview] DEBUG: Method called with stream_type={stream_type}")
        self._manage_nginx('start')
        
        # Ensure we have a session
        if not self.session:
            print("[start_preview] ERROR: No session available")
            raise RuntimeError("No session available. Call connect() first.")
        
        print(f"[start_preview] DEBUG: Session is available: {type(self.session)}")
        
        # Use the correct OSC commands/execute endpoint with the payload from api_payloads.json
        url = f"{self.base_url}/commands/execute"
        payload = json.loads(json.dumps(self.api_payloads["start_preview"]))
        
        print(f"[start_preview] DEBUG: Original payload: {payload}")
        
        # Modify the payload to include RTMP push URL and standard settings for 30 FPS
        if "parameters" in payload:
            # Update the stiching parameters to use 30 FPS
            if "stiching" in payload["parameters"]:
                payload["parameters"]["stiching"]["width"] = 1920   # 保持低解析度
                payload["parameters"]["stiching"]["height"] = 960   # 保持低解析度
                payload["parameters"]["stiching"]["framerate"] = 30  # 回到30 FPS
                payload["parameters"]["stiching"]["bitrate"] = 4000  # 回到標準位元率
                # Add the RTMP URL for the stitched stream - use host IP that camera can reach
                payload["parameters"]["stiching"]["liveUrl"] = f"rtmp://192.168.1.11:1935/live/preview"

            # Also update origin parameters for 30 FPS
            if "origin" in payload["parameters"]:
                payload["parameters"]["origin"]["framerate"] = 30  # 回到30 FPS
                payload["parameters"]["origin"]["bitrate"] = 8000  # 回到標準位元率

        print(f"[start_preview] DEBUG: Modified payload: {payload}")
        
        try:
            print(f"[start_preview] DEBUG: Current fingerprint: {self.fingerprint}")
            print(f"[start_preview] DEBUG: Sending startPreview command to {url}")
            
            response = await self.session.post(url, json=payload, headers=self._get_headers())
            response.raise_for_status()
            response_data = response.json()
            
            print(f"[start_preview] DEBUG: Full response: {response_data}")
            
            if response_data.get("state") == "done":
                print("[start_preview] SUCCESS: Preview started successfully.")
                return response_data
            else:
                print(f"[start_preview] ERROR: Preview failed with response: {response_data}")
                self._manage_nginx('stop')
                raise RuntimeError(f"Preview failed: {response_data}")
                
        except Exception as e:
            print(f"[start_preview] ERROR: Exception starting preview: {e}")
            self._manage_nginx('stop')
            raise

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
        self._manage_nginx(start=False) # Stop Nginx

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
        self._manage_nginx(start=True) # Ensure Nginx is running
        
        # Ensure we have a session
        if not self.session:
            raise RuntimeError("No session available. Call connect() first.")
        if not self.fingerprint:
            raise RuntimeError("尚未取得 fingerprint，請先 connect()")
            
        # Get the payload and modify the liveUrl to point to our local server
        payload = json.loads(json.dumps(self.api_payloads["start_live"]))
        if "parameters" in payload and "origin" in payload["parameters"]:
            payload["parameters"]["origin"]["liveUrl"] = "rtmp://127.0.0.1:1935/live/preview"
            payload["parameters"]["origin"]["bitrate"] = 8000  # 8Mbps for better quality
            payload["parameters"]["origin"]["framerate"] = 30  # Higher framerate
        
        cmd_url = f"{self.base_url}/commands/execute"
        try:
            logging.info(f"[start_live] Sending start_live command with payload: {payload}")
            response = await self.session.post(cmd_url, json=payload, headers=self._get_headers())
            response.raise_for_status()
            resp_data = response.json()
            
            logging.info(f"[start_live] Full response: {resp_data}")
            
            if resp_data.get("state") == "done":
                logging.info("[start_live] Live stream started successfully.")
                return resp_data
            else:
                logging.error(f"[start_live] Live stream failed with response: {resp_data}")
                self._manage_nginx('stop')
                raise RuntimeError(f"Live stream failed: {resp_data}")
                
        except Exception as e:
            logging.error(f"[start_live] Error starting live stream: {e}")
            self._manage_nginx('stop')
            raise

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
        """停止 origin live 串流，並嘗試關閉 nginx 服務"""
        if not self.fingerprint:
            return
        payload = self.api_payloads["stop_live"]
        url = f"{self.base_url}/commands/execute"
        async with aiohttp.ClientSession() as session:
            async with session.post(url, json=payload, headers=self._get_headers()) as resp:
                data = await resp.json(content_type=None)
                if data.get("state") != "done":
                    raise RuntimeError(f"stop_live 失敗: {data}")
        self._manage_nginx(start=False) # Stop Nginx
