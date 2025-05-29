import os
import asyncio
import base64
import io
import traceback
from typing import Optional, Dict, Any, Callable, List
import threading

# 完全使用 google.genai API
from google import genai
from google.genai import types
import cv2
import mss
import PIL.Image
import pyaudio

from shared_queue import log_queue_gemini

# --- Configuration ---
# IMPORTANT: Set your API key as an environment variable
# export GOOGLE_API_KEY="YOUR_API_KEY" or GEMINI_API_KEY="YOUR_API_KEY"
API_KEY = os.getenv("GOOGLE_API_KEY") or os.getenv("GEMINI_API_KEY") or "AIzaSyAdf5Mg-42Ccd6lON8S3Rr2kK5okWHV53Q"
if not API_KEY:
    log_queue_gemini.put("[GeminiClient] Error: GOOGLE_API_KEY or GEMINI_API_KEY environment variable not set.")

DEFAULT_MODEL = "gemini-1.5-flash"
LIVE_MODEL = "models/gemini-2.5-flash-preview-native-audio-dialog"

# Audio settings for Live API
FORMAT = pyaudio.paInt16
CHANNELS = 1
SEND_SAMPLE_RATE = 16000
RECEIVE_SAMPLE_RATE = 24000
CHUNK_SIZE = 1024

class GeminiClient:
    """統一的 Gemini API 客戶端，完全基於 google.genai API，支援傳統文字模式和 Live 互動模式"""
    
    def __init__(self, api_key=API_KEY):
        self.api_key = api_key
        self.current_model_name = DEFAULT_MODEL
        
        # Live API 相關屬性
        self.client = None
        self.live_session = None
        self.audio_loop = None
        self.live_mode_active = False
        self.live_event_loop = None
        self.live_thread = None
        
        # 配置 genai 客戶端
        self._configure()
        
        # 添加初始化日誌
        log_queue_gemini.put(f"[GeminiClient] Initialized with API key: {'***' + api_key[-10:] if api_key else 'None'}")

    def _configure(self):
        """配置 google.genai API 客戶端"""
        if not self.api_key:
            log_queue_gemini.put("[GeminiClient] Configuration skipped: API key is missing.")
            return
        try:
            self.client = genai.Client(
                api_key=self.api_key,
                http_options={"api_version": "v1beta"}
            )
            log_queue_gemini.put("[GeminiClient] Gemini API configured successfully.")
        except Exception as e:
            log_queue_gemini.put(f"[GeminiClient] Error configuring Gemini API: {e}")

    def set_model(self, model_name=DEFAULT_MODEL):
        """設定用於傳統文字生成的模型"""
        if not self.api_key:
            log_queue_gemini.put("[GeminiClient] Cannot set model: API key is missing.")
            return False
        try:
            self.current_model_name = model_name
            log_queue_gemini.put(f"[GeminiClient] Model set to: {model_name}")
            return True
        except Exception as e:
            log_queue_gemini.put(f"[GeminiClient] Error setting model '{model_name}': {e}")
            return False

    def generate_response(self, prompt):
        """使用 google.genai API 進行同步文字生成"""
        if not self.client:
            log_queue_gemini.put("[GeminiClient] Cannot generate response: Client not configured.")
            return "Error: Client not available."
        
        try:
            log_queue_gemini.put(f"[GeminiClient] Sending prompt to model {self.current_model_name}...")
            
            # 使用 google.genai API 生成內容
            response = self.client.models.generate_content(
                model=self.current_model_name,
                contents=prompt
            )
            
            log_queue_gemini.put(f"[GeminiClient] Received response from model.")
            
            # 提取回應文字
            if response and hasattr(response, 'text') and response.text:
                return response.text
            elif response and hasattr(response, 'candidates') and response.candidates:
                # 嘗試從 candidates 中提取文字
                for candidate in response.candidates:
                    if hasattr(candidate, 'content') and candidate.content:
                        if hasattr(candidate.content, 'parts') and candidate.content.parts:
                            for part in candidate.content.parts:
                                if hasattr(part, 'text') and part.text:
                                    return part.text
            else:
                log_queue_gemini.put(f"[GeminiClient] Unexpected response format: {response}")
                return "Error: Unexpected response format from API."

        except Exception as e:
            log_queue_gemini.put(f"[GeminiClient] Error generating response: {e}")
            return f"Error: API call failed. {e}"

    def list_available_models(self):
        """列出可用的 Gemini 模型，使用 google.genai API"""
        if not self.client:
            log_queue_gemini.put("[GeminiClient] Cannot list models: Client not configured.")
            return ["gemini-1.5-flash", "gemini-pro"]
        
        try:
            # 使用 google.genai API 列出模型
            models_response = self.client.models.list()
            
            if hasattr(models_response, 'models'):
                models = []
                for model in models_response.models:
                    if hasattr(model, 'name'):
                        model_name = model.name
                        # 過濾出 Gemini 模型
                        if 'gemini' in model_name.lower():
                            # 移除 'models/' 前綴
                            if model_name.startswith('models/'):
                                model_name = model_name[7:]
                            models.append(model_name)
                
                if models:
                    log_queue_gemini.put(f"[GeminiClient] Available models: {models}")
                    return models
            
            # 如果沒有獲取到模型，返回預設模型
            default_models = ["gemini-1.5-flash", "gemini-pro", "gemini-1.5-pro"]
            log_queue_gemini.put(f"[GeminiClient] Using default models: {default_models}")
            return default_models
            
        except Exception as e:
            log_queue_gemini.put(f"[GeminiClient] Error listing models: {e}")
            return ["gemini-1.5-flash", "gemini-pro"]

    def start_live_session(self, video_mode="none", voice_name="Zephyr",
                          response_modalities=None,
                          on_text_received: Optional[Callable[[str], None]] = None,
                          on_audio_received: Optional[Callable[[bytes], None]] = None):
        """
        啟動 Live 互動模式
        
        Args:
            video_mode: "camera", "screen", "none"
            voice_name: 語音名稱 (如 "Zephyr")
            response_modalities: 回應模式，預設為 ["AUDIO"]
            on_text_received: 接收到文字時的回調函數
            on_audio_received: 接收到音訊時的回調函數
        """
        log_queue_gemini.put(f"[GeminiClient] start_live_session called with video_mode={video_mode}")
        
        # 移除環境變數禁用檢查，允許 Live 模式啟動
        # 註解掉原本的禁用邏輯
        # if os.getenv('DISABLE_LIVE_MODE') == '1':
        #     log_queue_gemini.put("[GeminiClient] Live mode is disabled by environment variable.")
        #     return False
        
        if self.live_mode_active:
            log_queue_gemini.put("[GeminiClient] Live session already active.")
            return False

        if not self.api_key:
            log_queue_gemini.put("[GeminiClient] Cannot start live session: API key is missing.")
            return False

        if not self.client:
            log_queue_gemini.put("[GeminiClient] Cannot start live session: Client not configured.")
            return False

        try:
            log_queue_gemini.put("[GeminiClient] Starting Live session...")
            
            if response_modalities is None:
                response_modalities = ["AUDIO"]
            
            log_queue_gemini.put(f"[GeminiClient] Creating Live config with voice={voice_name}, modalities={response_modalities}")
            
            # 建立 Live 配置
            config = types.LiveConnectConfig(
                response_modalities=response_modalities,
                media_resolution="MEDIA_RESOLUTION_MEDIUM",
                speech_config=types.SpeechConfig(
                    voice_config=types.VoiceConfig(
                        prebuilt_voice_config=types.PrebuiltVoiceConfig(voice_name=voice_name)
                    )
                ),
                context_window_compression=types.ContextWindowCompressionConfig(
                    trigger_tokens=25600,
                    sliding_window=types.SlidingWindow(target_tokens=12800),
                ),
            )

            log_queue_gemini.put("[GeminiClient] Creating AudioLoop...")
            # 建立 AudioLoop 實例
            self.audio_loop = AudioLoop(
                client=self.client,
                config=config,
                video_mode=video_mode,
                on_text_received=on_text_received,
                on_audio_received=on_audio_received
            )

            log_queue_gemini.put("[GeminiClient] Starting Live session thread...")
            # 在新執行緒中啟動 Live session
            self.live_thread = threading.Thread(
                target=self._run_live_session_thread,
                daemon=True
            )
            self.live_thread.start()
            
            self.live_mode_active = True
            log_queue_gemini.put(f"[GeminiClient] Live session started successfully with video_mode: {video_mode}")
            return True

        except Exception as e:
            log_queue_gemini.put(f"[GeminiClient] Error starting live session: {e}")
            log_queue_gemini.put(f"[GeminiClient] Start live session traceback: {traceback.format_exc()}")
            return False

    def _run_live_session_thread(self):
        """在新執行緒中運行 Live session"""
        try:
            log_queue_gemini.put("[GeminiClient] Starting Live session thread...")
            # 建立新的事件循環
            self.live_event_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.live_event_loop)
            
            log_queue_gemini.put("[GeminiClient] Running AudioLoop...")
            # 運行 AudioLoop
            self.live_event_loop.run_until_complete(self.audio_loop.run())
            
        except Exception as e:
            log_queue_gemini.put(f"[GeminiClient] Live session thread error: {e}")
            log_queue_gemini.put(f"[GeminiClient] Full traceback: {traceback.format_exc()}")
        finally:
            self.live_mode_active = False
            if self.live_event_loop:
                self.live_event_loop.close()
            log_queue_gemini.put("[GeminiClient] Live session thread finished.")

    def stop_live_session(self):
        """停止 Live 互動模式"""
        if not self.live_mode_active:
            log_queue_gemini.put("[GeminiClient] No active live session to stop.")
            return False

        try:
            if self.audio_loop:
                self.audio_loop.stop()
            
            if self.live_event_loop and not self.live_event_loop.is_closed():
                # 在事件循環中安排停止
                asyncio.run_coroutine_threadsafe(
                    self._stop_live_session_async(), 
                    self.live_event_loop
                )

            # 等待執行緒結束
            if self.live_thread and self.live_thread.is_alive():
                self.live_thread.join(timeout=5.0)

            self.live_mode_active = False
            self.audio_loop = None
            self.live_session = None
            
            log_queue_gemini.put("[GeminiClient] Live session stopped.")
            return True

        except Exception as e:
            log_queue_gemini.put(f"[GeminiClient] Error stopping live session: {e}")
            return False

    async def _stop_live_session_async(self):
        """異步停止 Live session"""
        if self.live_session:
            try:
                await self.live_session.close()
            except:
                pass

    def send_text_to_live(self, text: str):
        """向 Live session 發送文字訊息"""
        if not self.live_mode_active or not self.audio_loop:
            log_queue_gemini.put("[GeminiClient] Cannot send text: Live session not active.")
            return False

        try:
            if self.live_event_loop and not self.live_event_loop.is_closed():
                asyncio.run_coroutine_threadsafe(
                    self.audio_loop.send_text(text),
                    self.live_event_loop
                )
                return True
        except Exception as e:
            log_queue_gemini.put(f"[GeminiClient] Error sending text to live session: {e}")
        
        return False

    def send_audio_to_live(self, audio_data: bytes):
        """向 Live session 發送音訊資料"""
        if not self.live_mode_active or not self.audio_loop:
            log_queue_gemini.put("[GeminiClient] Cannot send audio: Live session not active.")
            return False

        try:
            if self.live_event_loop and not self.live_event_loop.is_closed():
                asyncio.run_coroutine_threadsafe(
                    self.audio_loop.send_audio(audio_data),
                    self.live_event_loop
                )
                return True
        except Exception as e:
            log_queue_gemini.put(f"[GeminiClient] Error sending audio to live session: {e}")
        
        return False

    def send_image_to_live(self, image_data: Dict[str, Any]):
        """向 Live session 發送圖片資料"""
        if not self.live_mode_active or not self.audio_loop:
            log_queue_gemini.put("[GeminiClient] Cannot send image: Live session not active.")
            return False

        try:
            if self.live_event_loop and not self.live_event_loop.is_closed():
                asyncio.run_coroutine_threadsafe(
                    self.audio_loop.send_image(image_data),
                    self.live_event_loop
                )
                return True
        except Exception as e:
            log_queue_gemini.put(f"[GeminiClient] Error sending image to live session: {e}")
        
        return False

    def is_live_mode_active(self):
        """檢查 Live 模式是否啟動"""
        return self.live_mode_active

    def get_current_mode(self):
        """取得目前模式"""
        return "live" if self.live_mode_active else "text"


class AudioLoop:
    """完全基於 google.genai API 的 AudioLoop 類別"""
    
    def __init__(self, client, config, video_mode="none", 
                 on_text_received=None, on_audio_received=None):
        self.client = client
        self.config = config
        self.video_mode = video_mode
        self.on_text_received = on_text_received
        self.on_audio_received = on_audio_received
        
        self.audio_in_queue = None
        self.out_queue = None
        self.session = None
        
        self.audio_stream = None
        self.pya = pyaudio.PyAudio()
        
        self.running = False
        self.tasks = []

    async def run(self):
        """運行主要的 Live session"""
        try:
            self.running = True
            log_queue_gemini.put("[AudioLoop] Connecting to Live API...")
            
            async with (
                self.client.aio.live.connect(model=LIVE_MODEL, config=self.config) as session,
                asyncio.TaskGroup() as tg,
            ):
                self.session = session
                log_queue_gemini.put("[AudioLoop] Connected to Live API successfully!")
                
                self.audio_in_queue = asyncio.Queue()
                self.out_queue = asyncio.Queue(maxsize=5)
                
                # 建立各種任務
                self.tasks.append(tg.create_task(self.send_realtime()))
                self.tasks.append(tg.create_task(self.listen_audio()))
                
                if self.video_mode == "camera":
                    self.tasks.append(tg.create_task(self.get_frames()))
                elif self.video_mode == "screen":
                    self.tasks.append(tg.create_task(self.get_screen()))
                
                self.tasks.append(tg.create_task(self.receive_audio()))
                self.tasks.append(tg.create_task(self.play_audio()))
                
                log_queue_gemini.put("[AudioLoop] All tasks started, entering main loop...")
                
                # 等待任務完成或被取消
                await asyncio.Event().wait()  # 無限等待直到被外部停止

        except asyncio.CancelledError:
            log_queue_gemini.put("[AudioLoop] Session cancelled.")
        except Exception as e:
            log_queue_gemini.put(f"[AudioLoop] Session error: {e}")
            log_queue_gemini.put(f"[AudioLoop] Full traceback: {traceback.format_exc()}")
        finally:
            self.running = False
            if self.audio_stream:
                self.audio_stream.close()
            self.pya.terminate()
            log_queue_gemini.put("[AudioLoop] AudioLoop cleanup completed.")

    def stop(self):
        """停止 AudioLoop"""
        log_queue_gemini.put("[AudioLoop] Stopping AudioLoop...")
        self.running = False
        for task in self.tasks:
            if not task.done():
                task.cancel()

    async def send_text(self, text: str):
        """發送文字訊息"""
        if self.session:
            await self.session.send(input=text, end_of_turn=True)

    async def send_audio(self, audio_data: bytes):
        """發送音訊資料"""
        if self.out_queue:
            await self.out_queue.put({"data": audio_data, "mime_type": "audio/pcm"})

    async def send_image(self, image_data: Dict[str, Any]):
        """發送圖片資料"""
        if self.out_queue:
            await self.out_queue.put(image_data)

    async def send_realtime(self):
        """發送即時資料到 session"""
        while self.running:
            try:
                msg = await self.out_queue.get()
                if self.session:
                    await self.session.send(input=msg)
            except asyncio.CancelledError:
                break
            except Exception as e:
                log_queue_gemini.put(f"[AudioLoop] Error in send_realtime: {e}")

    async def listen_audio(self):
        """監聽音訊輸入"""
        try:
            mic_info = self.pya.get_default_input_device_info()
            self.audio_stream = await asyncio.to_thread(
                self.pya.open,
                format=FORMAT,
                channels=CHANNELS,
                rate=SEND_SAMPLE_RATE,
                input=True,
                input_device_index=mic_info["index"],
                frames_per_buffer=CHUNK_SIZE,
            )
            
            kwargs = {"exception_on_overflow": False} if __debug__ else {}
            
            while self.running:
                data = await asyncio.to_thread(
                    self.audio_stream.read, CHUNK_SIZE, **kwargs
                )
                await self.out_queue.put({"data": data, "mime_type": "audio/pcm"})
                
        except asyncio.CancelledError:
            pass
        except Exception as e:
            log_queue_gemini.put(f"[AudioLoop] Error in listen_audio: {e}")

    async def receive_audio(self):
        """接收音訊回應"""
        while self.running:
            try:
                turn = self.session.receive()
                async for response in turn:
                    if data := response.data:
                        self.audio_in_queue.put_nowait(data)
                        if self.on_audio_received:
                            self.on_audio_received(data)
                        continue
                    if text := response.text:
                        if self.on_text_received:
                            self.on_text_received(text)

                # 處理中斷情況
                while not self.audio_in_queue.empty():
                    self.audio_in_queue.get_nowait()
                    
            except asyncio.CancelledError:
                break
            except Exception as e:
                log_queue_gemini.put(f"[AudioLoop] Error in receive_audio: {e}")

    async def play_audio(self):
        """播放音訊"""
        try:
            stream = await asyncio.to_thread(
                self.pya.open,
                format=FORMAT,
                channels=CHANNELS,
                rate=RECEIVE_SAMPLE_RATE,
                output=True,
            )
            
            while self.running:
                bytestream = await self.audio_in_queue.get()
                await asyncio.to_thread(stream.write, bytestream)
                
        except asyncio.CancelledError:
            pass
        except Exception as e:
            log_queue_gemini.put(f"[AudioLoop] Error in play_audio: {e}")

    def _get_frame(self, cap):
        """擷取攝影機畫面"""
        ret, frame = cap.read()
        if not ret:
            return None
        
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = PIL.Image.fromarray(frame_rgb)
        img.thumbnail([1024, 1024])
        
        image_io = io.BytesIO()
        img.save(image_io, format="jpeg")
        image_io.seek(0)
        
        mime_type = "image/jpeg"
        image_bytes = image_io.read()
        return {"mime_type": mime_type, "data": base64.b64encode(image_bytes).decode()}

    async def get_frames(self):
        """持續擷取攝影機畫面"""
        try:
            cap = await asyncio.to_thread(cv2.VideoCapture, 0)
            
            while self.running:
                frame = await asyncio.to_thread(self._get_frame, cap)
                if frame is None:
                    break
                
                await asyncio.sleep(1.0)
                await self.out_queue.put(frame)
            
            cap.release()
            
        except asyncio.CancelledError:
            pass
        except Exception as e:
            log_queue_gemini.put(f"[AudioLoop] Error in get_frames: {e}")

    def _get_screen(self):
        """擷取螢幕畫面"""
        sct = mss.mss()
        monitor = sct.monitors[0]
        
        i = sct.grab(monitor)
        
        mime_type = "image/jpeg"
        image_bytes = mss.tools.to_png(i.rgb, i.size)
        img = PIL.Image.open(io.BytesIO(image_bytes))
        
        image_io = io.BytesIO()
        img.save(image_io, format="jpeg")
        image_io.seek(0)
        
        image_bytes = image_io.read()
        return {"mime_type": mime_type, "data": base64.b64encode(image_bytes).decode()}

    async def get_screen(self):
        """持續擷取螢幕畫面"""
        try:
            while self.running:
                frame = await asyncio.to_thread(self._get_screen)
                if frame is None:
                    break
                
                await asyncio.sleep(1.0)
                await self.out_queue.put(frame)
                
        except asyncio.CancelledError:
            pass
        except Exception as e:
            log_queue_gemini.put(f"[AudioLoop] Error in get_screen: {e}")
