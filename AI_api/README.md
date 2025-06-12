# Gemini API for PyQt 應用說明

## 1. API 端口設計理念

- **非阻塞**：所有長時間操作（如 AI 回覆）皆提供 async 介面，確保 UI 不會卡頓。
- **多 session 支援**：每個視窗/對話可建立獨立 client 實例，互不干擾。
- **Callback 機制**：Live 模式支援即時回傳文字/音訊，方便 UI 動態更新。
- **簡單易用**：API 介面簡潔，適合直接在 PyQt 的 signal/slot 或 async event loop 中使用。

---

## 2. 主要 API 端口

| 函式名稱                | 功能說明                                 | 適用場景                |
|------------------------|------------------------------------------|-------------------------|
| create_client          | 建立 GeminiClient 實例                   | 每個對話/視窗一個 client |
| list_models            | 取得可用模型清單                         | 模型下拉選單            |
| set_model              | 設定目前使用的模型                       | 切換模型                |
| text_chat_async        | 非同步文字對話，回傳 AI 回覆              | 文字聊天                |
| start_live             | 啟動 Live 模式，註冊 callback            | 語音/多模態互動         |
| stop_live              | 停止 Live 模式                           | 結束語音互動            |
| send_text_to_live      | 向 Live session 傳送文字                  | Live 模式下發送訊息     |
| is_live_active         | 查詢 Live 模式是否啟動                   | UI 狀態同步             |

---

## 3. PyQt 典型用法範例

### 3.1 文字聊天（非阻塞）

```python
from AI_api import create_client, list_models, set_model, text_chat_async
from PyQt5.QtCore import QThread, pyqtSignal, QObject
import asyncio

class ChatWorker(QThread):
    reply_ready = pyqtSignal(str)
    def __init__(self, client, prompt):
        super().__init__()
        self.client = client
        self.prompt = prompt

    def run(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        reply = loop.run_until_complete(text_chat_async(self.client, self.prompt))
        self.reply_ready.emit(reply)

# 在 PyQt UI 事件中
client = create_client()
set_model(client, "gemini-1.5-flash")

def on_user_send_text(prompt):
    worker = ChatWorker(client, prompt)
    worker.reply_ready.connect(update_ui_with_reply)
    worker.start()
```

### 3.2 Live 模式（即時語音/多模態）

```python
from AI_api import start_live, stop_live, send_text_to_live, is_live_active

def on_live_text(text):
    # 在 UI thread 安全地更新顯示
    ui.textBrowser.append(f"AI: {text}")

def on_live_audio(audio_bytes):
    # 播放音訊或顯示音訊波形
    pass

# 啟動 Live
start_live(client, on_text=on_live_text, on_audio=on_live_audio)

# 發送訊息到 Live
send_text_to_live(client, "你好，請即時回應")

# 停止 Live
stop_live(client)
```

---

## 4. 注意事項

- **thread-safe**：Live callback 會在背景 thread 執行，請用 Qt signal/slot 機制將資料傳回 UI thread。
- **async/await**：如需直接用 async/await，可考慮在 PyQt5/PyQt6 中用 QEventLoop 或 qasync。
- **多 client 支援**：每個對話/視窗可建立獨立 client，互不干擾。
- **Live session 狀態**：請用 `is_live_active` 查詢狀態，避免重複啟動/停止。

---

## 5. 進階建議

- 可將 client 實例封裝於 PyQt 的 QObject 子類，統一管理 signal/slot。
- 若需多模型切換，建議 UI 先 list_models，讓用戶選擇後再 set_model。

---

如需更進階的 async/context manager 或 PyQt 整合範例，請提出具體需求！

