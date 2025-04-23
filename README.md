# 🔐 Security Robot AI

本專案為一套模組化、可擴充的保全機器人 AI 系統，具備多執行緒架構、即時人臉辨識（模擬）、ReID 身份比對（模擬）、攝影機畫面顯示，並整合 PyQt 圖形化使用者介面與 **Google Gemini AI 互動**。

---

## 📂 專案目錄架構

```bash
Security_Robot_AI/
├── main.py               # 主程式入口
├── core.py               # 執行緒集中管理模組
├── workers.py            # 背景執行緒模組 (Camera, RTMP, Face, ReID, Gemini)
├── shared_queue.py       # 共享佇列與事件定義
├── ui.py                 # PyQt5 UI 啟動模組
├── ui_window.py          # UI 介面與邏輯 (ControlPanel 類別)
├── gemini_client.py      # Google Gemini API 互動客戶端
├── README.md             # 本說明文件
├── INSTA_config/         # (Optional) Insta360 configuration files
├── Old_Fuction/          # (Optional) Old test scripts
```

---

## ♻️ 模組說明

| 模組檔案         | 功能簡述 |
|------------------|----------|
| `main.py`        | 系統入口，呼叫核心與 UI 模組 |
| `core.py`        | 建立並啟動/停止所有背景執行緒 |
| `workers.py`     | 背景任務模組：<br/> - `camera_worker` 讀取攝影機畫面 <br/> - `rtmp_worker` 推送模擬影像 <br/> - `face_detector_worker` 模擬人臉擷取 <br/> - `reid_worker` 身分辨識模擬 <br/> - `gemini_worker` 處理 Gemini API 請求 |
| `shared_queue.py`| 定義模組共用的佇列與停止旗標 |
| `gemini_client.py`| 封裝 Gemini API 的呼叫邏輯 |
| `ui_window.py`   | PyQt UI 顯示邏輯（含攝影機畫面、日誌、Gemini 互動） |
| `ui.py`          | 啟動 UI 主介面應用 |

---

## 📆 依賴安裝

```bash
pip install PyQt5 opencv-python google-generativeai
```

**重要:** 你需要設定 Google Gemini API 金鑰。將你的 API 金鑰設定為環境變數 `GOOGLE_API_KEY`：

```bash
# Linux / macOS
export GOOGLE_API_KEY="YOUR_API_KEY"

# Windows (Command Prompt)
set GOOGLE_API_KEY=YOUR_API_KEY

# Windows (PowerShell)
$env:GOOGLE_API_KEY="YOUR_API_KEY"
```

或者，直接在 `gemini_client.py` 中修改 `API_KEY = "YOUR_API_KEY"`（較不建議）。

---

## ⚡️ 執行方式

```bash
python main.py
```

---

## ✨ 新增功能
- **Gemini AI 整合**:
    - 可在 UI 中選擇 Gemini 模型 (e.g., `gemini-1.5-flash`, `gemini-pro`)。
    - 輸入提示詞並發送給 Gemini API。
    - 在 UI 中顯示 Gemini 的回應。
    - Gemini 相關的日誌會顯示在獨立的 Log Box 中。
- **UI 改進**:
    - 使用 `QSplitter` 調整 Gemini 面板與主監控面板的寬度。
    - 優化 Log 更新機制。
    - 改善啟動/停止按鈕狀態管理。
    - 關閉視窗時自動停止背景執行緒。

## 🚀 發展方向（建議）
- [ ] 串接真實 YOLOv8 或人臉辨識模型取代模擬器
- [ ] 將識別記錄/Gemini 對話記錄寫入 SQLite/CSV
- [ ] 黑名單比對與警報機制
- [ ] 將 UI 轉為更複雜的多頁/分頁系統
- [ ] 錯誤處理與使用者回饋優化
- [ ] 使用設定檔管理 API Key 與模型名稱

---

Made with ❤️ for secure, modular, and scalable robot AI systems.