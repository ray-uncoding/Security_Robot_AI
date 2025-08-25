# Jetson Orin (L4T/JetPack) Dockerfile for Security_Robot_AI
FROM nvcr.io/nvidia/l4t-base:r36.2.0

# 設定非互動模式並指定時區，避免 apt-get 卡住
ENV TZ=Asia/Taipei
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# 安裝基本工具與依賴
RUN apt-get update && apt-get install -y \
    python3-pip python3-dev python3-opencv \
    libsm6 libxext6 libxrender-dev \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# 設定 python3 為預設
RUN ln -sf python3 /usr/bin/python

# 複製專案檔案
WORKDIR /workspace
COPY . /workspace

# 安裝 Python 依賴
RUN pip3 install --upgrade pip
RUN pip3 install -r requirements.txt

# 安裝 Jetson 專用 torch/torchvision（需根據 JetPack 版本調整 wheel 來源）
# RUN pip3 install --extra-index-url https://pypi.nvidia.com torch torchvision

# Jetson 需要的 CUDA/加速庫已內建於 l4t-base

CMD ["python3", "insta_api_test_preview.py"]
