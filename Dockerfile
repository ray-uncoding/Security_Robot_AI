# Jetson Orin (L4T/JetPack) Dockerfile for Security_Robot_AI
FROM nvcr.io/nvidia/l4t-base:r36.2.0

# 設定非互動模式並指定時區，避免 apt-get 卡住
ENV TZ=Asia/Taipei
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Add NVIDIA Tegra library paths for video codec and other GPU libraries
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,video
# Define build-time arg to avoid "UndefinedVar" warning, then set ENV
ARG LD_LIBRARY_PATH
ENV LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra-egl:$LD_LIBRARY_PATH

# The ENV LD_LIBRARY_PATH line will be removed. We will handle GPU access during 'docker run'.

# 安裝基本工具與依賴
RUN apt-get update && apt-get install -y \
    build-essential pkg-config cmake \
    python3-pip python3-dev python3-opencv \
    python3-pyqt5 \
    libsm6 libxext6 libxrender-dev \
    libxcb1 libx11-xcb1 libxcb-xinerama0 libxkbcommon-x11-0 \
    libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-randr0 \
    libxcb-render0 libxcb-shm0 libxcb-sync1 libxcb-xfixes0 \
    libxshmfence1 \
    ffmpeg \
    libportaudio2 portaudio19-dev libsndfile1-dev libasound2-dev \
    && rm -rf /var/lib/apt/lists/*

# 設定 python3 為預設
RUN ln -sf python3 /usr/bin/python

# 複製專案檔案
WORKDIR /workspace
COPY . /workspace

# 安裝 Python 依賴（先升級 pip / wheel / setuptools）
ENV PIP_NO_CACHE_DIR=1
RUN pip3 install --upgrade pip setuptools wheel
RUN pip3 install -r requirements.txt

# 安裝 Jetson 專用 torch/torchvision（需根據 JetPack 版本調整 wheel 來源）
# RUN pip3 install --extra-index-url https://pypi.nvidia.com torch torchvision

# Jetson 需要的 CUDA/加速庫已內建於 l4t-base

# 確保 Qt/XCB 後端使用 X11 並讓 OpenCV Qt 插件能找到正確的 plugin 目錄
ENV QT_QPA_PLATFORM=xcb
ENV QT_QPA_PLATFORM_PLUGIN_PATH=/usr/local/lib/python3.10/dist-packages/cv2/qt/plugins
ENV QT_DEBUG_PLUGINS=1
ENV LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}

# 將容器預設啟動改為 main.py（UI）
CMD ["python3", "main.py"]
