# GeminiClient 統一 API 文檔

## 概述

`GeminiClient` 是一個統一的 Google Gemini API 客戶端，整合了傳統文字生成功能和 Live 即時互動功能。設計目標是保持完全的向後相容性，同時提供強大的多模態即時互動能力。

## 核心特性

### ✅ 向後相容性
- 保留所有現有的同步 API 介面
- 現有程式碼無需修改即可繼續使用
- 支援所有 `google.generativeai` 的功能

### 🆕 新增功能
- Live API 即時音訊/視訊互動
- 統一的模式切換管理
- 多模態內容支援（文字、音訊、圖片）
- 異步事件處理

## API 參考

### 基本初始化

```python
from gemini_client import GeminiClient

# 使用環境變數中的 API key
client = GeminiClient()

# 或指定 API key
client = GeminiClient(api_key="your_api_key")
```

### 傳統文字模式 API（完全向後相容）

#### `list_available_models()` - 靜態方法
列出所有可用的 Gemini 模型。

```python
models = GeminiClient.list_available_models()
print(f"可用模型: {models}")
```

**返回值**: `List[str]` - 模型名稱列表

#### `set_model(model_name)`
設定用於文字生成的模型。

```python
success = client.set_model("gemini-1.5-flash")
```

**參數**:
- `model_name` (str): 模型名稱，預設為 "gemini-1.5-flash"

**返回值**: `bool` - 是否設定成功

#### `generate_response(prompt)`
生成文字回應（同步方法）。

```python
response = client.generate_response("Hello, how are you?")
print(response)
```

**參數**:
- `prompt` (str): 輸入提示

**返回值**: `str` - AI 生成的回應文字

### Live 互動模式 API（新功能）

#### `start_live_session()`
啟動 Live 即時互動會話。

```python
success = client.start_live_session(
    video_mode="camera",  # "camera", "screen", "none"
    voice_name="Zephyr",  # 語音名稱
    response_modalities=["AUDIO"],  # 回應模式
    on_text_received=text_callback,  # 文字回調
    on_audio_received=audio_callback  # 音訊回調
)
```

**參數**:
- `video_mode` (str): 視訊模式
  - `"camera"`: 使用攝影機
  - `"screen"`: 使用螢幕擷取
  - `"none"`: 僅音訊模式
- `voice_name` (str): AI 語音名稱，預設 "Zephyr"
- `response_modalities` (List[str]): 回應模態，預設 ["AUDIO"]
- `on_text_received` (Callable): 接收文字時的回調函數
- `on_audio_received` (Callable): 接收音訊時的回調函數

**返回值**: `bool` - 是否啟動成功

#### `stop_live_session()`
停止 Live 即時互動會話。

```python
success = client.stop_live_session()
```

**返回值**: `bool` - 是否停止成功

#### `send_text_to_live(text)`
向 Live 會話發送文字訊息。

```python
success = client.send_text_to_live("Hello from text!")
```

**參數**:
- `text` (str): 要發送的文字

**返回值**: `bool` - 是否發送成功

#### `send_audio_to_live(audio_data)`
向 Live 會話發送音訊資料。

```python
success = client.send_audio_to_live(audio_bytes)
```

**參數**:
- `audio_data` (bytes): PCM 格式的音訊資料

**返回值**: `bool` - 是否發送成功

#### `send_image_to_live(image_data)`
向 Live 會話發送圖片資料。

```python
image_data = {
    "mime_type": "image/jpeg",
    "data": base64_encoded_image
}
success = client.send_image_to_live(image_data)
```

**參數**:
- `image_data` (Dict): 包含 mime_type 和 base64 資料的字典

**返回值**: `bool` - 是否發送成功

### 狀態查詢 API

#### `is_live_mode_active()`
檢查 Live 模式是否啟動。

```python
is_active = client.is_live_mode_active()
```

**返回值**: `bool` - Live 模式是否啟動

#### `get_current_mode()`
取得目前運作模式。

```python
mode = client.get_current_mode()  # "text" 或 "live"
```

**返回值**: `str` - "text" 或 "live"

## 使用範例

### 範例 1: 傳統文字生成（向後相容）

```python
from gemini_client import GeminiClient

# 初始化客戶端
client = GeminiClient()

# 設定模型
client.set_model("gemini-1.5-flash")

# 生成回應
response = client.generate_response("什麼是人工智慧？")
print(response)
```

### 範例 2: Live 語音互動

```python
from gemini_client import GeminiClient

def on_text_received(text):
    print(f"AI 說: {text}")

def on_audio_received(audio_data):
    print(f"收到音訊: {len(audio_data)} bytes")

# 初始化客戶端
client = GeminiClient()

# 啟動 Live 會話（僅音訊）
client.start_live_session(
    video_mode="none",
    on_text_received=on_text_received,
    on_audio_received=on_audio_received
)

# 發送文字訊息
client.send_text_to_live("你好，我想聊天！")

# 運行一段時間...
import time
time.sleep(10)

# 停止會話
client.stop_live_session()
```

### 範例 3: Live 視訊互動

```python
from gemini_client import GeminiClient

# 初始化客戶端
client = GeminiClient()

# 啟動帶攝影機的 Live 會話
client.start_live_session(
    video_mode="camera",
    response_modalities=["AUDIO", "TEXT"]
)

# AI 可以看到攝影機畫面並進行即時互動
client.send_text_to_live("你看到了什麼？")

# 稍後停止
client.stop_live_session()
```

## 環境設定

### API Key 配置

支援多種 API key 配置方式：

```bash
# 方式 1: 環境變數 GOOGLE_API_KEY
export GOOGLE_API_KEY="your_api_key"

# 方式 2: 環境變數 GEMINI_API_KEY  
export GEMINI_API_KEY="your_api_key"

# 方式 3: 程式碼中直接設定
client = GeminiClient(api_key="your_api_key")
```

### 依賴套件

傳統文字模式只需要：
```bash
pip install google-generativeai
```

Live 模式額外需要：
```bash
pip install google-genai opencv-python pyaudio pillow mss
```

## 錯誤處理

所有方法都包含完善的錯誤處理機制：

- API key 缺失時會返回預設值或錯誤訊息
- 網路錯誤會被捕獲並記錄
- Live 會話異常會自動清理資源
- 所有錯誤都會透過 `log_queue_gemini` 記錄

## 技術架構

### 雙 API 整合
- **傳統模式**: 使用 `google.generativeai` (同步)
- **Live 模式**: 使用 `google.genai` (異步)

### 執行緒管理
- Live 模式在獨立執行緒中運行
- 使用 `asyncio` 處理即時資料流
- 線程安全的佇列通訊

### 資源管理
- 自動清理音訊/視訊資源
- 妥善處理會話生命週期
- 記憶體使用最佳化

## 遷移指南

### 從舊版本遷移

現有程式碼**無需修改**，新版本完全向後相容：

```python
# 舊程式碼 - 繼續正常工作
client = GeminiClient()
models = client.list_available_models()
client.set_model("gemini-1.5-flash")
response = client.generate_response("Hello")
```

### 新功能採用

可以逐步採用新功能：

```python
# 保持現有功能
response = client.generate_response("傳統問答")

# 新增 Live 功能
if not client.is_live_mode_active():
    client.start_live_session(video_mode="none")
    client.send_text_to_live("即時對話")
```

## 注意事項

1. **API Key**: Live 模式需要支援 Live API 的 Gemini API key
2. **權限**: 攝影機和麥克風需要系統權限
3. **網路**: Live 模式需要穩定的網路連接
4. **資源**: 視訊模式會消耗較多 CPU 和頻寬
5. **相容性**: 某些舊版套件可能不相容，建議使用最新版本

## 更新日誌

### v2.0.0 (統一版本)
- ✅ 整合 Live API 功能
- ✅ 保持完全向後相容性
- ✅ 新增多模態支援
- ✅ 改善錯誤處理和日誌記錄
- ✅ 統一配置管理
- ✅ 執行緒安全改善

### v1.0.0 (原版本)
- ✅ 基本文字生成功能
- ✅ 模型列表和切換
- ✅ 基本錯誤處理