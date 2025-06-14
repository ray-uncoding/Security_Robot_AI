# 🔐 Security Robot AI - 多鏡頭即時監控與 AI 互動系統

本專案旨在打造一套模組化、可擴展的保全機器人 AI 系統。系統核心功能包括：

*   **多鏡頭即時影像處理**：整合 Insta360 Pro 2 攝影機，透過 RTMP 協定進行六路原始鏡頭（`origin live` 模式）的即時串流。
*   **硬體加速影像管線**：利用 FFmpeg 搭配 NVIDIA GPU (CUVID/CUDA) 實現對多路高解析度（如 3.8K @ 24fps）H.264 視訊串流的硬體解碼與縮放，顯著降低 CPU 負載。
*   **即時人物檢測與身份辨識 (ReID)**：採用 YOLOv8 進行人物偵測，並結合 OSNet 模型提取人物特徵，實現跨鏡頭的身份追蹤與識別。
*   **動態身份資料庫管理**：為每個識別出的人物動態維護特徵資料庫，支援新身份註冊與舊身份更新。
*   **PyQt 圖形化使用者介面 (GUI)**：即時顯示拼接後的多鏡頭畫面、人物檢測框、身份標註以及與 AI 的互動訊息。
*   **Google Gemini AI 互動**：整合 Google Gemini API，實現與系統的自然語言對話與指令控制。
*   **多執行緒/多程序架構**：確保影像處理、AI 分析、UI 更新等任務高效並行運作，提升系統回應速度與處理能力。

本系統適用於需要即時監控、人物追蹤以及智慧互動的安防場景、學術研究與原型開發。

---

## 📂 專案目錄架構 (概覽)

```
Security_Robot_AI/
├── main.py                     # 系統主程式入口
├── insta_api_test.py           # Insta360 相機 API 與 RTMP 串流測試腳本
├── requirements.txt            # Python 相依套件清單
├── README.md                   # 本專案說明文件
├── yolov8n.pt                  # YOLOv8 預訓練模型權重 (範例)
│
├── AI_api/                     # Google Gemini AI 相關模組
│   ├── gemini_client.py        # Gemini API 客戶端
│   └── ...
│
├── Insta_OpenCV/               # Insta360 串流與 OpenCV 處理模組
│   ├── utils/
│   │   └── frame_receiver.py   # 核心：FFmpeg 拉流、硬體加速處理與影像幀提供者
│   ├── config/
│   │   └── api_payloads.json   # Insta360 相機 API 指令 (如 start_live 參數)
│   │   └── settings.json       # (可能存在的) 其他設定檔
│   ├── README.md               # Insta360 RTMP 管線詳細說明
│   └── ...
│
├── core/                       # 核心執行緒管理與共享資源
│   ├── core.py                 # 執行緒/程序集中管理
│   └── shared_queue.py         # 跨模組共享佇列與事件定義
│
├── workers/                    # 各類背景工作者模組
│   └── workers.py              # 定義 CameraStreamWorker, FFmpegWorker, ReIDWorker, GeminiWorker 等
│
├── reid/                       # 人物身份辨識 (ReID) 相關模組
│   ├── detector.py             # YOLOv8 人物偵測器
│   ├── reid_model.py           # OSNet ReID 特徵提取模型
│   ├── dynamic_db.py           # 動態身份特徵資料庫管理
│   └── reid_manager.py         # ReID 流程管理器
│
├── ui/                         # PyQt 使用者介面模組
│   ├── ui_window_V2.py         # (或 ui_window.py) PyQt UI 視窗與顯示邏輯
│   └── ui.py                   # UI 主介面啟動邏輯
│
├── nginx 1.7.11.3 Gryphon/     # Nginx RTMP 伺服器 (用於串流中轉)
│   ├── conf/nginx.conf         # Nginx 設定檔
│   └── ...
│
├── ffmpeg-master-latest-win64-gpl-shared/ # FFmpeg 執行檔與函式庫
│   ├── bin/ffmpeg.exe
│   └── ...
│
└── config/                     # 全域設定檔或模型權重
    ├── osnet_x0_25_imagenet.pth # OSNet 預訓練模型權重 (範例)
    └── yolov8n.pt              # (重複參考，通常放一處)
```

*註：上述目錄結構根據目前觀察到的檔案進行推斷，可能與實際情況略有差異。*

---

## ♻️ 系統功能與核心技術

1.  **Insta360 Pro 2 多鏡頭 RTMP 串流 (`Insta_OpenCV/` & `insta_api_test.py`)**
    *   透過相機 API (`api_payloads.json`) 控制啟動 `origin live` 模式，推送六路 3840x2880 @ 24fps, 10Mbps H.264 串流 (參數可調)。
    *   使用 Nginx (`nginx 1.7.11.3 Gryphon/`) 作為 RTMP 中繼伺服器。

2.  **FFmpeg 硬體加速影像處理 (`Insta_OpenCV/utils/frame_receiver.py`)**
    *   從 Nginx 拉取六路 RTMP 串流。
    *   **硬體解碼**: `-c:v h264_cuvid` (NVIDIA GPU)。
    *   **硬體縮放**: `scale_cuda` (NVIDIA GPU)，例如縮放至 640x360。
    *   **硬體下載與色彩空間轉換**: `hwdownload, format=nv12, format=bgr24` (GPU->CPU，BGR24 轉換可能部分在 CPU)。
    *   將處理後的 BGR24 影像幀透過管道傳輸給 Python。

3.  **YOLOv8 人物偵測 (`reid/detector.py`)**
    *   使用 `yolov8n.pt` (或其他版本) 模型即時偵測畫面中的人物。
    *   輸出人物邊界框 (bounding box)。

4.  **OSNet ReID 特徵提取 (`reid/reid_model.py`)**
    *   對 YOLO 偵測到的人物圖像，使用 OSNet (`osnet_x0_25_imagenet.pth`) 模型提取高維度特徵向量 (如 512 維)。

5.  **動態身份資料庫與匹配 (`reid/dynamic_db.py`, `reid/reid_manager.py`)**
    *   為每個獨立個體維護一個特徵向量集合。
    *   使用餘弦相似度等方法比對新提取的特徵與資料庫中的已知特徵，進行身份確認或新身份註冊。

6.  **PyQt 圖形化使用者介面 (`ui/`)**
    *   使用 PyQt5/PySide 設計 GUI。
    *   即時顯示由六個鏡頭畫面拼接而成的監控畫面 (例如 2x3 網格的 1920x720)。
    *   在畫面上繪製人物偵測框及標註身份 ID。
    *   提供與 Google Gemini AI 的文字輸入與聊天歷史顯示區域。

7.  **Google Gemini AI 互動 (`AI_api/gemini_client.py`)**
    *   連接 Google Gemini API。
    *   處理使用者輸入的自然語言，獲取 AI 回應，並顯示在 UI 上。
    *   可能用於查詢系統狀態、發出簡單指令或進行場景理解的問答。

8.  **多執行緒/程序管理 (`core/core.py`, `workers/workers.py`)**
    *   將相機串流讀取、FFmpeg 處理、ReID 分析、Gemini 通訊、UI 更新等任務分配到不同的執行緒或程序中。
    *   使用共享佇列 (`core/shared_queue.py`) 進行跨執行緒/程序的資料交換與同步。

---

## 🛠️ 設定與執行

1.  **環境準備**：
    *   安裝 Python (建議 3.8+)。
    *   安裝 FFmpeg 並確保其路徑在系統環境變數中，或者在程式中指定其執行路徑 (如 `ffmpeg-master-latest-win64-gpl-shared/bin/ffmpeg.exe`)。
    *   安裝 Nginx 並設定 RTMP 模組 (如 `nginx 1.7.11.3 Gryphon/` 中的配置)。
    *   確保擁有支援 CUDA 的 NVIDIA GPU 及對應的驅動程式。

2.  **安裝 Python 相依套件**：
    ```bash
    pip install -r requirements.txt
    ```
    (請確保 `requirements.txt` 包含 `torch`, `torchvision`, `torchaudio`, `opencv-python`, `PyQt5` 或 `PySide6`, `google-generativeai` 等核心套件)。

3.  **設定 Google Gemini API 金鑰**：
    將您的 API 金鑰設定為環境變數，或在 `AI_api/gemini_client.py` (或其他設定檔) 中配置。
    ```bash
    # Linux/macOS
    export GOOGLE_API_KEY="YOUR_API_KEY"
    # Windows (PowerShell)
    $env:GOOGLE_API_KEY="YOUR_API_KEY"
    ```

4.  **設定 Insta360 相機串流參數**：
    *   編輯 `Insta_OpenCV/config/api_payloads.json` 中的 `start_live` 部分，確認 `liveUrl` 指向您的 Nginx RTMP 伺服器地址。
    *   根據需求調整解析度 (`width`, `height`)、幀率 (`framerate`)、位元率 (`bitrate`) 等參數。目前設定為 3840x2880 @ 24fps, 10Mbps。

5.  **啟動 Nginx RTMP 伺服器**：
    進入 `nginx 1.7.11.3 Gryphon/` 目錄，執行 `nginx.exe`。

6.  **執行主程式**：
    ```bash
    python main.py
    ```
    或者，如果您想單獨測試相機串流與 FFmpeg 管線：
    ```bash
    python insta_api_test.py
    ```

---

## ✨ 系統特色

*   **端到端多鏡頭處理**：從相機原始串流到最終 AI 分析與 UI 展示的完整解決方案。
*   **高效能硬體加速**：充分利用 GPU 進行繁重的影像解碼與縮放任務。
*   **模組化設計**：各功能模組（串流、偵測、ReID、AI、UI）相對獨立，易於維護、升級與擴展。
*   **即時性**：針對即時監控場景進行優化，力求低延遲。
*   **智慧互動**：整合大型語言模型，賦予系統更自然的交互能力。

---

## 🚀 未來發展方向 (To-Do / 考慮點)

*   **模型優化與量化**：
    *   對 YOLOv8 和 OSNet 模型進行剪枝、量化 (如 TensorRT) 以在邊緣裝置或資源受限環境下提升效能。
    *   探索更輕量級或針對特定場景優化的人物偵測與 ReID 模型。
*   **多相機/多節點管理**：
    *   支援同時管理來自多台 Insta360 攝影機或其他類型攝影機的串流。
    *   設計分佈式處理架構，將負載分散到多個處理節點。
*   **資料持久化與查詢**：
    *   將偵測結果、ReID 特徵、身份標籤、重要事件、AI 對話記錄等儲存到資料庫 (如 SQLite, PostgreSQL, MongoDB)。
    *   提供歷史資料查詢與分析功能。
*   **進階 AI 功能**：
    *   行為分析：識別異常行為、徘徊、入侵等。
    *   跨鏡頭無縫追蹤與全景軌跡生成。
    *   更複雜的場景理解與事件描述。
*   **使用者體驗與介面優化**：
    *   更精細的 UI 設計，提供更多控制選項與狀態顯示。
    *   支援設定檔管理，方便使用者自訂參數。
    *   提供警報通知機制 (聲音、郵件、App 推送等)。
*   **系統強固性與錯誤處理**：
    *   完善各模組的錯誤捕捉與恢復機制。
    *   增加詳細的日誌記錄功能，方便問題追蹤與排查。
*   **部署方案**：
    *   提供 Docker 容器化部署方案。
    *   研究在嵌入式平台 (如 NVIDIA Jetson 系列) 上的部署與優化。
*   **安全性**：
    *   確保 API 金鑰、串流資料等敏感資訊的安全。

---

This README provides a comprehensive overview of the Security Robot AI project, its architecture, functionalities, and setup instructions. For more detailed information on specific modules, please refer to the README files within their respective directories (e.g., `Insta_OpenCV/README.md`).