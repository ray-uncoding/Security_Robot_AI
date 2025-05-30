# API 配額檢查工具使用指南

## 概述
`check_api_quota.py` 是一個診斷工具，用於檢查 Gemini API 金鑰的有效性和配額狀態。

## 使用方法

### 1. 執行配額檢查
```bash
python check_api_quota.py
```

### 2. 工具功能
- **API 金鑰驗證**：檢查金鑰是否有效
- **模型列表**：顯示可用的 Gemini 模型
- **基本測試**：執行簡單的文字生成測試
- **配額診斷**：識別配額超限問題
- **解決方案指南**：提供詳細的配額管理建議

### 3. 環境變數設定
確保已設定 API 金鑰：
```bash
# Linux/Mac
export GOOGLE_API_KEY='your-api-key'

# Windows
set GOOGLE_API_KEY=your-api-key
```

### 4. 常見問題解決

#### 配額超限錯誤（錯誤碼 1011）
- 等待配額重置（通常為 1 分鐘或 24 小時）
- 前往 [Google AI Studio](https://aistudio.google.com/) 檢查配額使用情況
- 考慮升級到付費方案

#### API 金鑰無效
- 確認金鑰是否正確複製
- 檢查環境變數是否正確設定
- 確認金鑰是否已啟用

#### Live API 不支援
- Live API 需要特殊權限
- 確認您的帳號是否有 Live API 存取權限
- 可能需要申請 Live API 測試權限

## 系統改進內容

### 1. 錯誤處理增強
- `gemini_client.py`：加入配額錯誤識別和詳細錯誤訊息
- `workers.py`：實施配額錯誤冷卻機制（60秒）
- `ui_window.py`：配額錯誤會以特殊樣式顯示

### 2. 使用者體驗改善
- 明確的錯誤類型識別（配額、認證、模型等）
- 詳細的解決方案建議
- UI 中的視覺化錯誤提示

### 3. 重試機制
- 配額錯誤後自動進入冷卻期
- 避免無效的重複請求
- 提供剩餘等待時間提示