from .gemini_client import GeminiClient
import asyncio

# 高階 API 端口，適合 UI 框架直接調用

def create_client() -> GeminiClient:
    """建立一個新的 GeminiClient 實例"""
    return GeminiClient()


def list_models(client: GeminiClient) -> list:
    """查詢可用模型清單"""
    return client.list_available_models()


def set_model(client: GeminiClient, model_name: str) -> bool:
    """設定目前使用的模型"""
    return client.set_model(model_name)


async def text_chat_async(client: GeminiClient, prompt: str) -> str:
    """非同步文字對話（不阻塞 UI）"""
    loop = asyncio.get_event_loop()
    return await loop.run_in_executor(None, client.generate_response, prompt)


def start_live(client: GeminiClient, *, video_mode="none", voice_name="Zephyr", on_text=None, on_audio=None, mic_index=None) -> bool:
    """啟動 Live 語音/多模態模式，註冊 callback"""
    return client.start_live_session(
        video_mode=video_mode,
        voice_name=voice_name,
        on_text_received=on_text,
        on_audio_received=on_audio,
        mic_index=mic_index
    )


def stop_live(client: GeminiClient) -> bool:
    """停止 Live 模式"""
    return client.stop_live_session()


def send_text_to_live(client: GeminiClient, text: str) -> bool:
    """向 Live session 傳送文字訊息"""
    return client.send_text_to_live(text)


def is_live_active(client: GeminiClient) -> bool:
    """查詢 Live 模式是否啟動"""
    return client.is_live_mode_active()

# --- Usage Example ---
if __name__ == "__main__":
    import time
    import threading

    # 建立 client
    client = create_client()
    print("可用模型：", list_models(client))
    set_model(client, "gemini-1.5-flash")

    # 非同步文字對話
    async def chat():
        reply = await text_chat_async(client, "你好，請自我介紹")
        print("AI 回覆：", reply)
    asyncio.run(chat())

    # Live 模式 callback 範例
    def on_text(text):
        print("[Live] AI 語音回覆：", text)
    def on_audio(audio_bytes):
        print(f"[Live] 收到音訊 {len(audio_bytes)} bytes")

    # 啟動 Live
    print("啟動 Live 模式...")
    start_live(client, on_text=on_text, on_audio=on_audio)
    time.sleep(10)  # Demo: 讓 Live 跑 10 秒
    print("停止 Live 模式...")
    stop_live(client)
    print("Live 狀態：", is_live_active(client))
