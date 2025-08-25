# Jetson OpenCV Docker Demo

這個專案展示如何在 NVIDIA Jetson 平台上，透過 Docker 容器執行一個使用 OpenCV 的 Python 應用程式，並存取主機的 USB 攝影機。

## 執行步驟

### 1. 登入 NVIDIA Container Registry (NGC)

許多 Jetson 的基礎 Docker 映像 (l4t-base) 都需要經過身份驗證才能拉取。如果未登入，在 build 的第一步就會遇到 `not found` 的錯誤。

a. **取得 NGC API 金鑰：**
   - 前往 [NVIDIA NGC 網站](https://ngc.nvidia.com) 並登入。
   - 點擊右上角個人資料 > "Setup" > "Get API Key"。
   - 產生並複製您的 API 金鑰。

b. **使用 Docker 登入：**
   在終端機中執行以下指令，將 `YOUR_API_KEY` 替換為您剛剛複製的金鑰。
   ```bash
   docker login nvcr.io -u '$oauthtoken' -p 'YOUR_API_KEY'
   ```
   看到 `Login Succeeded` 即表示成功。

### 2. 建置 Docker 映像 (Build)

登入後，在專案根目錄執行以下指令來建置映像。我們使用的基礎映像為 `nvcr.io/nvidia/l4t-base:r36.2.0`。
```bash
docker build -t demo-opencv:jp60dp .
```

### 3. 執行 Docker 容器 (Run)

為了讓容器內的 OpenCV 程式可以：
1.  存取主機的 USB 攝影機 (`/dev/video0`)。
2.  在主機的桌面上顯示 GUI 視窗。

需要加上數個參數來執行容器。

a. **開放 X11 存取權限：**
   在**主機**終端機執行，暫時允許所有本地連線在您的桌面上繪製視窗。
   ```bash
   xhost +
   ```

b. **執行容器：**
   ```bash
   docker run --runtime nvidia -it --rm \
       --device /dev/video0 \
       -e DISPLAY=$DISPLAY \
       -v /tmp/.X11-unix/:/tmp/.X11-unix \
       demo-opencv:jp60dp
   ```
   此時應該會跳出攝影機的即時影像視窗。按下 `ESC` 鍵即可關閉。

c. **(建議) 恢復 X11 存取權限：**
   程式測試完畢後，為了安全，建議將權限恢復。
   ```bash
   xhost -
   ```

## 疑難排解 (Troubleshooting)

以下是我們在此專案中遇到的問題與解決方法摘要：

1.  **問題：** `docker build` 失敗，錯誤訊息為 `failed to resolve source metadata ... not found`。
    -   **原因：** Docker 無法拉取 `Dockerfile` 中指定的基礎映像 `nvcr.io/nvidia/l4t-base:xxx`。這通常是因為：
        1.  該映像版本標籤不存在。
        2.  需要登入 NGC 才能存取，但尚未登入。
    -   **解決方案：** 執行上述**步驟 1**，登入 NGC。我們最終使用的基礎映像版本為 `r36.2.0`。

2.  **問題：** 容器成功執行，但程式錯誤 `can't open camera by index` 或 `開不起 /dev/video0`。
    -   **原因：** Docker 容器預設與主機硬體隔離，無法直接存取 USB 攝影機設備。
    -   **解決方案：** 在 `docker run` 指令中加入 `--device /dev/video0`，將攝影機設備掛載到容器中。

3.  **問題：** 攝影機成功開啟，但程式錯誤 `Authorization required` 或 `Can't initialize GTK backend`。
    -   **原因：** 容器沒有權限在主機的桌面上開啟 GUI 視窗 (X11 授權失敗)。
    -   **解決方案：**
        1.  在 `docker run` 指令中加入 `-e DISPLAY=$DISPLAY` 和 `-v /tmp/.X11-unix/:/tmp/.X11-unix`，將顯示環境掛載進容器。
        2.  在執行 `docker run` **之前**，先在主機上執行 `xhost +` 來暫時停用存取控制。



git config --global user.name "你的GitHub名稱"
git config --global user.email "你註冊GitHub的email"