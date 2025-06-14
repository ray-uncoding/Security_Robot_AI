# Insta360 Pro 2 多鏡頭 RTMP 串流管線說明

本文檔旨在說明使用 Insta360 Pro 2 攝影機，透過 RTMP 進行多鏡頭串流，並由 FFmpeg 處理後，最終在 Python/OpenCV 中顯示的整個管線流程、影像品質（解析度）、幀率（FPS）以及中間經歷的各種轉換。

## 管線階段概覽

整個流程大致可分為三個主要階段：

1.  **攝影機推流階段**：Insta360 Pro 2 將各鏡頭的影像編碼後推送到 Nginx RTMP 伺服器。
2.  **串流拉取與處理階段**：FFmpeg 從 Nginx 拉取 RTMP 串流，進行解碼、縮放等處理，並將處理後的影像幀傳輸給 Python。
3.  **影像顯示階段**：Python/OpenCV 接收影像幀，進行拼接並透過 `cv2.imshow` 顯示。

---

## 詳細流程與參數

### 1. 攝影機推流階段 (Insta360 Pro 2 -> Nginx RTMP 伺服器)

*   **攝影機型號**：Insta360 Pro 2
*   **直播模式**：`origin live` (相機推送六個獨立的原始魚眼鏡頭影像串流)
*   **API 設定檔**：`config/api_payloads.json` 中的 `start_live` 指令
    *   **推流畫質 (每個鏡頭)**：
        *   寬度 (`width`): **3840** 像素
        *   高度 (`height`): **2880** 像素
    *   **推流 FPS (每個鏡頭)**：
        *   幀率 (`framerate`): **24 FPS**
    *   **推流位元率 (每個鏡頭)**：
        *   位元率 (`bitrate`): **10000 kbps** (即 10 Mbps)
    *   **視訊編碼 (`mime`)**: **H.264**
*   **此階段發生的主要轉換**：
    1.  **影像擷取**：相機的六個感光元件分別擷取光學影像。
    2.  **影像訊號處理 (ISP)**：相機內部對原始影像進行基礎處理（如白平衡、曝光調整等，具體取決於相機設定）。
    3.  **H.264 視訊編碼**：將 ISP 處理後的各路影像分別使用 H.264 編碼器壓縮，目標解析度為 3840x2880，目標幀率為 24 FPS，目標位元率為 10 Mbps。
    4.  **RTMP 封裝與推送**：將 H.264 編碼後的視訊資料封裝成 RTMP (Real-Time Messaging Protocol) 串流，透過網路推送到指定的 Nginx RTMP 伺服器。Nginx 主要作為中繼分發，通常不改變串流內容。

### 2. 串流拉取與處理階段 (Nginx RTMP 伺服器 -> FFmpeg -> Python)

*   **FFmpeg 拉流來源**：從 Nginx RTMP 伺服器拉取六路獨立的 RTMP 串流 (例如 `rtmp://<nginx_ip>:<port>/live/origin1` 至 `origin6`)。
    *   **FFmpeg 接收畫質 (每個鏡頭)**：**3840x2880**
    *   **FFmpeg 接收 FPS (每個鏡頭)**：**24 FPS**
*   **FFmpeg 內部處理轉換** (針對每個鏡頭串流，指令邏輯定義於 `utils/frame_receiver.py` 中的 `FrameReceiver` 類別)：
    1.  **RTMP 解封裝與網路接收**：FFmpeg 建立連線，接收 RTMP 串流資料。
    2.  **H.264 硬體解碼 (`-c:v h264_cuvid`)**：
        *   輸入：H.264 編碼的 3840x2880 @ 24fps 視訊流。
        *   處理：使用 NVIDIA GPU (CUVID) 進行硬體解碼。
        *   輸出：解碼後的原始視訊幀（例如 YUV 格式），儲存於 GPU 記憶體中。畫質仍為 3840x2880，幀率理論上為 24 FPS。
    3.  **GPU 硬體縮放 (`scale_cuda=w={target_width}:h={target_height}:format=nv12`)**：
        *   輸入：GPU 記憶體中的 3840x2880 原始視訊幀。
        *   `target_width` 和 `target_height` 由 `FrameReceiver` 初始化參數決定 (預設為 640 和 360)。
        *   處理：使用 NVIDIA GPU (CUDA) 進行硬體縮放。
        *   輸出：縮放後的視訊幀，儲存於 GPU 記憶體中，解析度變為 **640x360** (預設情況)，色彩格式為 NV12。幀率理論上仍為 24 FPS。
    4.  **硬體下載與色彩空間轉換 (`hwdownload,format=nv12,format=bgr24`)**：
        *   `hwdownload`: 將 GPU 記憶體中的 640x360 (NV12 格式) 視訊幀下載到 CPU 主記憶體。
        *   `format=nv12`: 指定從 GPU 下載的原始像素格式。
        *   `format=bgr24`: (此步驟為 CPU 上的軟體轉換) FFmpeg 將 NV12 格式的影像轉換為 OpenCV 常用的 BGR24 格式。
        *   輸出：CPU 主記憶體中的 **640x360** BGR24 格式視訊幀。幀率理論上仍為 24 FPS。
    5.  **輸出至 Python (`-f rawvideo pipe:`)**: FFmpeg 將最終處理完畢的 BGR24 原始視訊幀透過標準輸出管道 (pipe) 傳輸給 Python 腳本。

*   **FFmpeg 輸出給 Python 的畫質 (每個鏡頭)**：**640x360** (預設情況)，BGR24 格式。
*   **FFmpeg 輸出給 Python 的 FPS (每個鏡頭)**：**理論上限為 24 FPS**。實際幀率取決於 GPU 的解碼與縮放能力、CPU 的下載與色彩轉換能力。若系統負載過高，FFmpeg 可能會發生掉幀，或導致傳輸給 Python 的速度減慢。

### 3. 影像顯示階段 (Python/OpenCV)

*   **Python 腳本接收**：從 FFmpeg 的管道中讀取每個鏡頭的 640x360 BGR24 格式的原始視訊幀。
*   **此階段發生的主要轉換/處理**：
    1.  **影像幀入隊/出隊** (如果使用了如 `shared_queue.py` 中的佇列機制)。
    2.  **影像拼接**：使用 OpenCV 的函式 (如 `cv2.vconcat`, `cv2.hconcat`) 將六個獨立的 640x360 畫面拼接成一個較大的網格畫面 (例如，2x3 的網格最終畫面解析度為 1920x720)。
    3.  **畫面顯示 (`cv2.imshow`)**：將拼接後的單一大畫面顯示在 OpenCV 的視窗中。

*   **OpenCV 最終顯示的畫質 (拼接後)**：取決於拼接方式。例如，若為 2x3 網格，則為 **1920x720**。單個子畫面的有效解析度仍為 640x360。
*   **OpenCV 最終顯示的 FPS (拼接後)**：**不會超過 24 FPS**。實際顯示的幀率會受到整個處理鏈路中最低效能環節的限制，包括 GPU 解碼/縮放瓶頸、CPU 處理（下載、色彩轉換、Python 邏輯、OpenCV 拼接與顯示）瓶頸。**此數值建議在 Python 腳本中實時計測以獲得準確結果。**

---

## 效能考量

*   **GPU 瓶頸**：同時處理六路高解析度 (3.8K) 視訊的硬體解碼與縮放對 GPU 效能（尤其是像 GTX 1060 這樣的消費級顯示卡）是極大的考驗，很容易達到 100% 使用率，成為整體效能的主要瓶頸。
*   **CPU 負載**：雖然大部分解碼與縮放工作由 GPU 完成，但資料從 GPU 到 CPU 的傳輸、色彩空間轉換 (如果 `format=bgr24` 是軟體完成)、Python 腳本本身的執行、OpenCV 的圖像操作（如拼接）以及最終的畫面顯示都會消耗 CPU 資源。
*   **網路頻寬與穩定性**：確保相機到 Nginx 以及 Nginx 到 FFmpeg 處理機器的網路連線具有足夠的頻寬和穩定性，以避免因網路問題導致的串流品質下降或中斷。

---

## 相關設定檔

*   攝影機 API 推流參數：`Insta_OpenCV/config/api_payloads.json`
*   FFmpeg 拉流與處理指令邏輯：`Insta_OpenCV/utils/frame_receiver.py` (主要在 `FrameReceiver` 類別的 `ffmpeg_cmd` 生成部分)

---

希望這份文件能幫助您清晰地了解整個串流管線的運作細節。
