import os
import asyncio
import base64
import io
import traceback
from typing import Optional, Dict, Any, Callable, List
import threading
import numpy as np # MODIFIED

# 完全使用 google.genai API
from google import genai
from google.genai import types
import cv2
import mss
import PIL.Image
import pyaudio

from core.shared_queue import log_queue_gemini

# --- Configuration ---
# IMPORTANT: Set your API key as an environment variable
# export GOOGLE_API_KEY="YOUR_API_KEY" or GEMINI_API_KEY="YOUR_API_KEY"
API_KEY = os.getenv("GOOGLE_API_KEY") or os.getenv("GEMINI_API_KEY")
if not API_KEY:
    log_queue_gemini.put("[GeminiClient] Error: GOOGLE_API_KEY or GEMINI_API_KEY environment variable not set.")

DEFAULT_MODEL = "gemini-1.5-flash"
LIVE_MODEL = "models/gemini-2.5-flash-preview-native-audio-dialog"  # 正確的 Live API 模型

# Audio settings for Live API
FORMAT = pyaudio.paInt16
CHANNELS = 1
SEND_SAMPLE_RATE = 16000
RECEIVE_SAMPLE_RATE = 24000
CHUNK_SIZE = 1024
# SILENCE_THRESHOLD = 500 # 可調整的靜音閾值 (RMS) # MODIFIED # Replaced by ENERGY_THRESHOLD
ENERGY_THRESHOLD = 100 # 可調整的能量閾值 (平均絕對值) # MODIFIED
MIN_SILENCE_CHUNKS_TO_STOP_SENDING = 3 # 連續多少個靜音塊後停止發送 # MODIFIED
MIN_VOICE_CHUNKS_TO_START_SENDING = 1 # 連續多少個語音塊後開始發送 # MODIFIED

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
            error_msg = str(e)
            log_queue_gemini.put(f"[GeminiClient] Error generating response: {error_msg}")
            
            # 檢查是否為配額超限錯誤
            if self._is_quota_error(error_msg):
                quota_error_msg = self._get_quota_error_message()
                log_queue_gemini.put(f"[GeminiClient] {quota_error_msg}")
                return f"配額錯誤: {quota_error_msg}"
            
            # 檢查是否為認證錯誤
            elif "invalid api key" in error_msg.lower() or "api key not valid" in error_msg.lower():
                return "錯誤: API 金鑰無效。請檢查您的 GOOGLE_API_KEY 或 GEMINI_API_KEY 環境變數。"
            
            # 檢查是否為模型不存在
            elif "model not found" in error_msg.lower():
                return f"錯誤: 找不到模型 '{self.current_model_name}'。請使用有效的模型名稱。"
            
            # 其他錯誤
            return f"錯誤: API 呼叫失敗。{error_msg}"

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
                          on_audio_received: Optional[Callable[[bytes], None]] = None,
                          mic_index=None):
        """
        啟動 Live 互動模式
        
        Args:
            video_mode: "camera", "screen", "none"
            voice_name: 語音名稱 (如 "Zephyr")
            response_modalities: 回應模式，預設為 ["AUDIO"]
            on_text_received: 接收到文字時的回調函數
            on_audio_received: 接收到音訊時的回調函數
        """
        log_queue_gemini.put(f"[GeminiClient THREAD_ID:{threading.get_ident()}] start_live_session called with video_mode={video_mode}") # MODIFIED
        
        # 移除環境變數禁用檢查，允許 Live 模式啟動
        # 註解掉原本的禁用邏輯
        # if os.getenv('DISABLE_LIVE_MODE') == '1':
        #     log_queue_gemini.put("[GeminiClient] Live mode is disabled by environment variable.")
        #     return False
        
        if self.live_mode_active:
            log_queue_gemini.put(f"[GeminiClient THREAD_ID:{threading.get_ident()}] Live session already active. Current audio_loop: {id(self.audio_loop) if self.audio_loop else 'None'}") # MODIFIED
            return False

        if not self.api_key:
            log_queue_gemini.put(f"[GeminiClient THREAD_ID:{threading.get_ident()}] Cannot start live session: API key is missing.") # MODIFIED
            return False

        if not self.client:
            log_queue_gemini.put(f"[GeminiClient THREAD_ID:{threading.get_ident()}] Cannot start live session: Client not configured.") # MODIFIED
            return False

        try:
            log_queue_gemini.put(f"[GeminiClient THREAD_ID:{threading.get_ident()}] Starting Live session...") # MODIFIED
            
            if response_modalities is None:
                response_modalities = ["AUDIO"]
            
            log_queue_gemini.put(f"[GeminiClient THREAD_ID:{threading.get_ident()}] Creating Live config with voice={voice_name}, modalities={response_modalities}") # MODIFIED
            
            # 診斷：驗證模型是否存在
            log_queue_gemini.put(f"[DIAGNOSIS THREAD_ID:{threading.get_ident()}] Attempting to use Live model: {LIVE_MODEL}") # MODIFIED
            
            # 診斷：檢查可用模型列表
            try:
                available_models = self.list_available_models()
                log_queue_gemini.put(f"[DIAGNOSIS THREAD_ID:{threading.get_ident()}] Available models: {available_models}") # MODIFIED
                live_model_exists = any('live' in model.lower() or 'dialog' in model.lower() for model in available_models)
                log_queue_gemini.put(f"[DIAGNOSIS THREAD_ID:{threading.get_ident()}] Live model variants found: {live_model_exists}") # MODIFIED
            except Exception as model_check_error:
                log_queue_gemini.put(f"[DIAGNOSIS THREAD_ID:{threading.get_ident()}] Error checking available models: {model_check_error}") # MODIFIED
            
            # 診斷：驗證配置參數
            log_queue_gemini.put(f"[DIAGNOSIS THREAD_ID:{threading.get_ident()}] Config parameters:") # MODIFIED
            log_queue_gemini.put(f"  - response_modalities: {response_modalities}")
            log_queue_gemini.put(f"  - voice_name: {voice_name}")
            log_queue_gemini.put(f"  - video_mode: {video_mode}")
            
            # 建立 Live 配置（簡化版本以排除參數問題）
            try:
                # 先嘗試最簡配置
                config = types.LiveConnectConfig(
                    response_modalities=response_modalities,
                    speech_config=types.SpeechConfig(
                        voice_config=types.VoiceConfig(
                            prebuilt_voice_config=types.PrebuiltVoiceConfig(voice_name=voice_name)
                        )
                    )
                )
                log_queue_gemini.put(f"[DIAGNOSIS THREAD_ID:{threading.get_ident()}] Basic Live config created successfully") # MODIFIED
                
                # 如果基本配置成功，再添加進階設定
                try:
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
                    log_queue_gemini.put(f"[DIAGNOSIS THREAD_ID:{threading.get_ident()}] Full Live config created successfully") # MODIFIED
                except Exception as advanced_config_error:
                    log_queue_gemini.put(f"[DIAGNOSIS THREAD_ID:{threading.get_ident()}] Advanced config failed, using basic config: {advanced_config_error}") # MODIFIED
                    
            except Exception as basic_config_error:
                log_queue_gemini.put(f"[DIAGNOSIS THREAD_ID:{threading.get_ident()}] Basic config creation failed: {basic_config_error}") # MODIFIED
                raise

            log_queue_gemini.put(f"[GeminiClient THREAD_ID:{threading.get_ident()}] Creating AudioLoop. Current audio_loop: {id(self.audio_loop) if self.audio_loop else 'None'}") # MODIFIED
            # 建立 AudioLoop 實例
            self.audio_loop = AudioLoop(
                client=self.client,
                config=config,
                video_mode=video_mode,
                on_text_received=on_text_received,
                on_audio_received=on_audio_received,
                mic_index=mic_index
            )
            log_queue_gemini.put(f"[GeminiClient THREAD_ID:{threading.get_ident()}] New AudioLoop instance created with ID: {id(self.audio_loop)}") # MODIFIED

            log_queue_gemini.put(f"[GeminiClient THREAD_ID:{threading.get_ident()}] Starting Live session thread...") # MODIFIED
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
            error_msg = str(e)
            log_queue_gemini.put(f"[GeminiClient] Error starting live session: {error_msg}")
            log_queue_gemini.put(f"[GeminiClient] Start live session traceback: {traceback.format_exc()}")
            
            # 檢查是否為配額超限錯誤
            if self._is_quota_error(error_msg):
                quota_error_msg = self._get_quota_error_message()
                log_queue_gemini.put(f"[GeminiClient] Live API 配額錯誤: {quota_error_msg}")
            
            # 檢查是否為模型不支援錯誤
            elif "model not found" in error_msg.lower() or "not supported" in error_msg.lower():
                log_queue_gemini.put("[GeminiClient] Live API 可能未啟用或模型不支援。請確認您的帳號有 Live API 存取權限。")
            
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
                    self.audio_loop.send_text_message(text),
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
    
    def _is_quota_error(self, error_msg: str) -> bool:
        """檢查是否為配額超限錯誤。
        
        Args:
            error_msg: 錯誤訊息字串
            
        Returns:
            bool: 如果是配額錯誤則返回 True，否則返回 False
        """
        error_msg_lower = error_msg.lower()
        quota_indicators = [
            "quota",
            "rate limit",
            "too many requests",
            "resource exhausted",
            "1011",  # 配額錯誤碼
            "429",   # HTTP 429 Too Many Requests
            "limit exceeded",
            "quota exceeded"
        ]
        
        return any(indicator in error_msg_lower for indicator in quota_indicators)
    
    def _get_quota_error_message(self) -> str:
        """獲取配額錯誤的詳細說明。
        
        Returns:
            str: 配額錯誤的詳細說明和解決建議
        """
        return (
            "您的 API 配額已用盡。請採取以下措施：\n"
            "1. 等待配額重置（通常為每分鐘或每日限制）\n"
            "2. 檢查 Google AI Studio 中的配額使用情況：https://aistudio.google.com/\n"
            "3. 如果是免費帳號，考慮升級到付費方案以獲得更高配額\n"
            "4. 減少 API 呼叫頻率或優化您的使用模式\n"
            "5. 使用 check_api_quota.py 工具來診斷問題"
        )

    @staticmethod
    def list_input_devices():
        """回傳所有可用的麥克風裝置清單（index, name）"""
        p = pyaudio.PyAudio()
        devices = []
        for i in range(p.get_device_count()):
            info = p.get_device_info_by_index(i)
            if info.get('maxInputChannels', 0) > 0:
                devices.append({'index': i, 'name': info['name']})
        p.terminate()
        return devices


class AudioLoop:
    """完全基於 google.genai API 的 AudioLoop 類別"""
    
    def __init__(self, client, config, video_mode="none",
                 on_text_received=None, on_audio_received=None, mic_index=None):
        log_queue_gemini.put(f"[AudioLoop THREAD_ID:{threading.get_ident()}] __init__ called. Object ID: {id(self)}") # MODIFIED
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
        self.sending_audio = True # 新增狀態，初始時發送音訊 # MODIFIED
        self.silent_chunks_count = 0 # MODIFIED
        self.voice_chunks_count = 0 # MODIFIED
        self.is_gemini_speaking = asyncio.Event() # MODIFIED
        self.mic_index = mic_index

    async def run(self):
        """運行主要的 Live session"""
        try:
            self.running = True
            log_queue_gemini.put("[AudioLoop] Starting Live session run...")
            
            # 記錄連接參數
            log_queue_gemini.put(f"[AudioLoop] Connecting with model: {LIVE_MODEL}")
            log_queue_gemini.put(f"[AudioLoop] Video mode: {self.video_mode}")
            
            async with (
                self.client.aio.live.connect(model=LIVE_MODEL, config=self.config) as session,
                asyncio.TaskGroup() as tg,
            ):
                self.session = session
                log_queue_gemini.put("[AudioLoop] Connected to Live API successfully!")
                
                # 初始化佇列
                self.audio_in_queue = asyncio.Queue()
                self.out_queue = asyncio.Queue(maxsize=5)
                
                # 建立所有任務
                send_text_task = tg.create_task(self.send_text())
                tg.create_task(self.send_realtime())
                tg.create_task(self.listen_audio())
                
                if self.video_mode == "camera":
                    tg.create_task(self.get_frames())
                elif self.video_mode == "screen":
                    tg.create_task(self.get_screen())
                
                tg.create_task(self.receive_audio())
                tg.create_task(self.play_audio())
                
                log_queue_gemini.put("[AudioLoop] All tasks started successfully")
                
                # 等待 send_text_task 完成（當使用者輸入 'q' 時）
                await send_text_task
                raise asyncio.CancelledError("User requested exit")

        except asyncio.CancelledError:
            log_queue_gemini.put("[AudioLoop] Session cancelled.")
        except ExceptionGroup as EG:
            if self.audio_stream:
                self.audio_stream.close()
            log_queue_gemini.put(f"[AudioLoop] ExceptionGroup error: {EG}")
            traceback.print_exception(EG)
        except Exception as e:
            log_queue_gemini.put(f"[AudioLoop] Session error: {e}")
            log_queue_gemini.put(f"[AudioLoop] Full traceback: {traceback.format_exc()}")
        finally:
            self.running = False
            if self.audio_stream:
                self.audio_stream.close()
            log_queue_gemini.put("[AudioLoop] AudioLoop cleanup completed.")

    def stop(self):
        """停止 AudioLoop"""
        log_queue_gemini.put("[AudioLoop] Stopping AudioLoop...")
        self.running = False
        self.on_text_received = None # 確保回調被清理
        self.on_audio_received = None # 同樣清理音訊回調以保持一致性
        for task in self.tasks:
            if not task.done():
                task.cancel()

    async def send_text(self):
        """等待使用者輸入並發送文字（參考官方範例）"""
        while True:
            text = await asyncio.to_thread(
                input,
                "message > ",
            )
            if text.lower() == "q":
                break
            await self.session.send(input=text or ".", end_of_turn=True)

    async def send_text_message(self, text: str):
        """直接發送文字訊息"""
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
                    # if msg is END_OF_TURN_MARKER: # MODIFIED - REMOVED
                    #     log_queue_gemini.put(f"[AudioLoop DEBUG] send_realtime: Sending end_of_turn=True to API.") # MODIFIED - REMOVED
                    #     await self.session.send(end_of_turn=True) # MODIFIED - REMOVED
                    # else: # MODIFIED - REMOVED
                    await self.session.send(input=msg) # MODIFIED
            except asyncio.CancelledError:
                break
            except Exception as e:
                log_queue_gemini.put(f"[AudioLoop] Error in send_realtime: {e}")

    async def listen_audio(self):
        """監聽音訊輸入"""
        try:
            if self.mic_index is not None:
                mic_info = self.pya.get_device_info_by_index(self.mic_index)
            else:
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
                
                # 基礎靜音檢測 # MODIFIED
                audio_segment = np.frombuffer(data, dtype=np.int16) # MODIFIED
                if audio_segment.size == 0: # MODIFIED
                    energy = 0 # 如果數據為空，能量為0 # MODIFIED
                else: # MODIFIED
                    energy = np.mean(np.abs(audio_segment)) # 使用平均絕對值作為能量 # MODIFIED
                
                # 不需要再檢查 nan，因為 abs 和 mean 不太可能產生 nan，除非輸入本身是 nan
                # if np.isnan(energy): # 檢查 energy 是否為 nan
                #     log_queue_gemini.put(f"[AudioLoop] Energy is nan, treating as silence. Original data length: {len(data)}")
                #     energy = 0 # 如果是 nan，視為靜音

                # 當 Gemini 正在說話時，我們不應該發送任何來自麥克風的音訊
                # 並且我們應該重置靜音/語音塊計數，以確保 Gemini 說完後能正確檢測使用者語音
                if self.is_gemini_speaking.is_set(): # MODIFIED
                    if self.sending_audio:
                        log_queue_gemini.put(f"[AudioLoop] Gemini is speaking, suppressing mic input.") # MODIFIED
                        self.sending_audio = False
                    # 重置計數器，以便在 Gemini 停止講話後，VAD 可以重新評估
                    self.silent_chunks_count = 0 # MODIFIED
                    self.voice_chunks_count = 0
                elif energy < ENERGY_THRESHOLD: # MODIFIED
                    self.silent_chunks_count += 1 # MODIFIED
                    self.voice_chunks_count = 0 # 重置語音計數 # MODIFIED
                    if self.silent_chunks_count >= MIN_SILENCE_CHUNKS_TO_STOP_SENDING: # MODIFIED
                        if self.sending_audio: # MODIFIED
                            log_queue_gemini.put(f"[AudioLoop] Silence detected (Energy: {energy:.2f}), stopping audio send.") # MODIFIED
                            self.sending_audio = False # MODIFIED
                            # await self.out_queue.put(END_OF_TURN_MARKER) # MODIFIED - REMOVED
                else: # MODIFIED
                    self.voice_chunks_count += 1 # MODIFIED
                    self.silent_chunks_count = 0 # 重置靜音計數 # MODIFIED
                    if not self.sending_audio and self.voice_chunks_count >= MIN_VOICE_CHUNKS_TO_START_SENDING: # MODIFIED
                        log_queue_gemini.put(f"[AudioLoop] Voice detected (Energy: {energy:.2f}), resuming audio send.") # MODIFIED
                        self.sending_audio = True # MODIFIED
                
                if self.sending_audio: # MODIFIED
                    await self.out_queue.put({"data": data, "mime_type": "audio/pcm"}) # MODIFIED
                # 即使不發送，也讓佇列知道有資料，避免可能的阻塞 (如果 out_queue 有大小限制且下游依賴 get) # MODIFIED
                # 或者，如果下游不依賴固定頻率的 get，則靜音時可以完全不 put # MODIFIED

            if self.audio_stream: # 確保在退出循環後關閉流 # MODIFIED
                self.audio_stream.close() # MODIFIED
                
        except asyncio.CancelledError: # MODIFIED
            if self.audio_stream: # MODIFIED
                self.audio_stream.close() # MODIFIED
            pass # MODIFIED
        except Exception as e: # MODIFIED
            if self.audio_stream: # MODIFIED
                self.audio_stream.close() # MODIFIED
            log_queue_gemini.put(f"[AudioLoop] Error in listen_audio: {e}") # MODIFIED
            log_queue_gemini.put(f"[AudioLoop] Full traceback for listen_audio error: {traceback.format_exc()}") # MODIFIED

    async def receive_audio(self):
        """接收音訊回應"""
        while self.running:
            try:
                if not self.session:
                    log_queue_gemini.put(f"[AudioLoop DEBUG] receive_audio: No session, sleeping.") # MODIFIED
                    await asyncio.sleep(0.1)
                    continue
                
                log_queue_gemini.put(f"[AudioLoop DEBUG] receive_audio: Calling self.session.receive()") # MODIFIED
                turn = self.session.receive()
                log_queue_gemini.put(f"[AudioLoop DEBUG] receive_audio: self.session.receive() returned. Type of turn: {type(turn)}") # MODIFIED

                processed_response = False # MODIFIED
                async for response in turn:
                    processed_response = True # MODIFIED
                    log_queue_gemini.put(f"[AudioLoop DEBUG] receive_audio: Got response in turn. Has data: {response.data is not None}, Has text: {response.text is not None}") # MODIFIED
                    if data := response.data:
                        log_queue_gemini.put(f"[AudioLoop] Received audio data: {len(data)} bytes")# MODIFIED
                        self.audio_in_queue.put_nowait(data)
                        if self.on_audio_received:
                            self.on_audio_received(data)
                        continue
                    if text := response.text:
                        log_queue_gemini.put(f"[AudioLoop] Received text: {text}")
                        if self.running and self.on_text_received: # 增加 self.running 檢查
                            self.on_text_received(text)
                
                if not processed_response: # MODIFIED
                    log_queue_gemini.put(f"[AudioLoop DEBUG] receive_audio: No responses iterated in 'turn'.") # MODIFIED

                # 處理中斷情況 - 清空音訊佇列
                # while not self.audio_in_queue.empty(): # MODIFIED
                #     self.audio_in_queue.get_nowait() # MODIFIED
                    
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
                try:
                    bytestream = await asyncio.wait_for(self.audio_in_queue.get(), timeout=0.2) # Increased timeout slightly
                    if not self.is_gemini_speaking.is_set():
                        self.is_gemini_speaking.set()
                        log_queue_gemini.put(f"[AudioLoop] Playback started, Gemini speaking event SET.")
                    
                    await asyncio.to_thread(stream.write, bytestream)
                    self.audio_in_queue.task_done()

                except asyncio.TimeoutError:
                    # This timeout means the audio_in_queue has been empty for a short while.
                    # If Gemini was speaking, we can assume it has finished.
                    if self.is_gemini_speaking.is_set():
                        self.is_gemini_speaking.clear()
                        log_queue_gemini.put(f"[AudioLoop] Playback queue empty, Gemini speaking event CLEARED.")
                    continue
                except asyncio.CancelledError:
                    log_queue_gemini.put(f"[AudioLoop] play_audio task cancelled.")
                    if self.is_gemini_speaking.is_set():
                        self.is_gemini_speaking.clear()
                    raise
                except Exception as e:
                    log_queue_gemini.put(f"[AudioLoop] Error in play_audio inner loop: {e}")
                    if self.is_gemini_speaking.is_set():
                        self.is_gemini_speaking.clear()
                    # Depending on the error, might want to break or continue
                    break
            
            # Final cleanup if loop exits
            if self.is_gemini_speaking.is_set():
                self.is_gemini_speaking.clear()
                log_queue_gemini.put(f"[AudioLoop] Clearing speaking flag on play_audio task exit.")

        except asyncio.CancelledError:
            log_queue_gemini.put(f"[AudioLoop] play_audio task fully cancelled (outer).")
            if self.is_gemini_speaking.is_set(): self.is_gemini_speaking.clear()
            pass # Propagate cancellation if necessary, or handle
        except Exception as e:
            log_queue_gemini.put(f"[AudioLoop] Error in play_audio setup or outer exception: {e}")
            if self.is_gemini_speaking.is_set(): self.is_gemini_speaking.clear()

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
