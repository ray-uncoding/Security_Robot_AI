# 🔐 Security Robot AI

本專案為一套模組化、可擴充的保全機器人 AI 系統，結合即時人物檢測、身份辨識與動態資料庫管理，並整合 PyQt 圖形化使用者介面與 Google Gemini AI 互動功能。系統採用多執行緒架構，實現高效能的即時處理，適用於學術研究與實際應用場景。

---

## 📂 專案目錄架構

```bash
Security_Robot_AI/
├── [main.py](http://_vscodecontentref_/0)               # 主程式入口
├── core/                 # 核心模組
│   ├── core.py           # 執行緒集中管理模組
│   ├── shared_queue.py   # 共享佇列與事件定義
├── workers/              # 背景執行緒模組
│   ├── workers.py        # 各類背景任務 (Camera, RTMP, Face, ReID, Gemini)
├── reid/                 # ReID 模組
│   ├── detector.py       # YOLO 檢測器
│   ├── reid_model.py     # ReID 特徵提取模型
│   ├── dynamic_db.py     # 動態資料庫管理
│   ├── reid_manager.py   # ReID 管理器
├── ui/                   # 使用者介面模組
│   ├── ui_window.py      # PyQt UI 顯示邏輯
│   ├── ui.py             # UI 主介面啟動
├── gemini_client.py      # Google Gemini API 互動客戶端
├── [README.md](http://_vscodecontentref_/1)             # 本說明文件
```

♻️ 系統功能
模組檔案	功能簡述
main.py	系統入口，呼叫核心與 UI 模組
core.py	建立並啟動/停止所有背景執行緒
workers.py	背景任務模組：<br/> - camera_worker 讀取攝影機畫面 <br/> - rtmp_worker 推送影像 <br/> - face_detector_worker 人物檢測 <br/> - reid_worker 身份辨識 <br/> - gemini_worker 處理 Gemini API 請求
shared_queue.py	定義模組共用的佇列與停止旗標
detector.py	使用 YOLO 模型進行人物檢測
reid_model.py	使用 ReID 模型提取特徵向量
dynamic_db.py	動態資料庫管理身份數據
reid_manager.py	整合檢測、特徵提取與身份比對
ui_window.py	PyQt UI 顯示邏輯（含攝影機畫面、日誌、Gemini 互動）
gemini_client.py	封裝 Gemini API 的呼叫邏輯
📖 核心技術
1. YOLO 檢測
使用 YOLO 模型進行即時人物檢測，返回檢測框與裁剪的臉部圖像。
支援多目標檢測，並將結果傳遞至身份辨識模組。
2. ReID 身份辨識
使用 ReID 模型提取臉部特徵，並與動態資料庫進行比對。
動態資料庫支援身份註冊與更新，實現高效的身份管理。
3. 動態資料庫
使用餘弦相似度進行特徵比對，並根據閾值決定是否註冊新身份。
支援多特徵存儲與更新，提升比對準確性。
4. PyQt 使用者介面
實現即時畫面顯示，並繪製檢測框與身份標籤。
整合 Google Gemini AI，支援自然語言互動。
📦 依賴安裝
請確保已安裝以下依賴：

重要: 你需要設定 Google Gemini API 金鑰。將你的 API 金鑰設定為環境變數 GOOGLE_API_KEY：

⚡️ 執行方式
✨ 系統特色
即時檢測與辨識：
使用 YOLO 模型進行即時人物檢測。
使用 ReID 模型進行身份辨識，並結合動態資料庫進行身份管理。
模組化設計：
採用多執行緒架構，實現高效能的即時處理。
各模組獨立運作，便於維護與擴展。
圖形化介面：
使用 PyQt 實現直觀的使用者介面。
支援即時畫面顯示與 Google Gemini AI 互動。
🚀 發展方向
<input disabled="" type="checkbox"> 支援多攝影機畫面處理。
<input disabled="" type="checkbox"> 將識別記錄與 Gemini 對話記錄寫入資料庫。
<input disabled="" type="checkbox"> 增加黑名單比對與警報機制。
<input disabled="" type="checkbox"> 優化 YOLO 與 ReID 模型的推理效率。
<input disabled="" type="checkbox"> 增強錯誤處理與使用者回饋功能。
Made with ❤️ for secure, modular, and scalable robot AI systems. ```