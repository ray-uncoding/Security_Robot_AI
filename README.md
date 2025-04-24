# 🔐 Security Robot AI

本專案為一套模組化、可擴充的保全機器人 AI 系統，結合即時人物檢測、身份辨識與動態資料庫管理，並整合 PyQt 圖形化使用者介面與 Google Gemini AI 互動功能。系統採用多執行緒架構，實現高效能的即時處理，適用於學術研究與實際應用場景。

---

## 📂 專案目錄架構

```bash
Security_Robot_AI/
├── main.py               # 主程式入口
├── core/                 # 核心模組
│   ├── core.py           # 執行緒集中管理模組
│   ├── shared_queue.py   # 共享佇列與事件定義
├── workers/              # 背景執行緒模組
│   └── workers.py        # 各類背景任務 (Camera, RTMP, Face, ReID, Gemini)
├── reid/                 # ReID 模組
│   ├── detector.py       # YOLO 檢測器
│   ├── reid_model.py     # ReID 特徵提取模型
│   ├── dynamic_db.py     # 動態資料庫管理
│   └── reid_manager.py   # ReID 管理器
├── ui/                   # 使用者介面模組
│   ├── ui_window.py      # PyQt UI 顯示邏輯
│   └── ui.py             # UI 主介面啟動
├── gemini_client.py      # Google Gemini API 互動客戶端
├── requirements.txt      # 相依套件清單
├── README.md             # 本說明文件
```

---

## ♻️ 系統功能概覽

| 模組 | 功能 |
|------|------|
| `main.py` | 系統主入口，整合核心模組與 UI 控制 |
| `core.py` | 建立並控制所有背景執行緒 |
| `shared_queue.py` | 定義跨模組的共用佇列與事件觸發器 |
| `workers.py` | 定義各背景工作任務：攝影機串流、RTMP 推送、人臉偵測、ReID、Gemini 對話 |
| `detector.py` | 使用 YOLOv8 進行人物偵測 |
| `reid_model.py` | 使用 OSNet 模型萃取人物特徵向量 |
| `dynamic_db.py` | 管理每個 ID 的特徵資料，支援動態更新與匹配 |
| `reid_manager.py` | 負責整合偵測與特徵比對邏輯 |
| `ui_window.py` | PyQt 界面顯示攝影機畫面、身份標註、AI 回覆訊息等 |
| `gemini_client.py` | 管理與 Google Gemini 的對話互動 |

---

## 📖 核心技術模組

### 1. YOLO 人物偵測
- 使用 YOLOv8 輕量模型即時辨識畫面中人物
- 回傳邊界框與裁剪人物區塊

### 2. ReID 身份辨識
- 使用 OSNet 模型擷取人物特徵（512 維向量）
- 與資料庫比對後決定身份（或新註冊 ID）

### 3. 動態資料庫管理
- 每個人保留 N 筆特徵（如 sliding window）
- 使用餘弦相似度進行快速比對與身份決策

### 4. PyQt 圖形化介面
- 顯示即時畫面、標註框與身份資訊
- 整合 Gemini 回覆區域進行語意互動

---

## 📦 安裝與執行

1. 安裝相依套件：
```bash
pip install -r requirements.txt
```

2. 設定 Google Gemini API：
```bash
export GOOGLE_API_KEY=your_api_key
```

3. 執行系統：
```bash
python main.py
```

---

## ✨ 系統特色

- **即時檢測與辨識**：高效 YOLO + 快速 ReID 模組
- **多執行緒架構**：保證 UI 與背景模組同步運作
- **動態資料管理**：提升辨識準確率與記憶體控制
- **自然語言互動**：支援 Google Gemini 對話，擴展智慧反應能力
- **模組分離架構**：可依據需求單獨調整 ReID / Gemini / Stream 等功能模組

---

## 🚀 發展方向（To-do）

- [ ] 支援多攝影機畫面同步處理
- [ ] 將辨識與對話記錄寫入本地或雲端資料庫
- [ ] 黑名單比對與警報提示功能
- [ ] 優化 YOLO/ReID 模型效能與部署模式（含 CPU / Jetson Nano 版本）
- [ ] 增加使用者操作回饋與錯誤提示機制
- [ ] 整合 face + body 雙重 ReID 模型架構
- [ ] 撰寫單元測試與模組測試案例（提高維護性）
- [ ] 設計更完整的狀態管理與多階段處理流程
- [ ] 撰寫外部開發 API 與使用者 Plugin 機制

---