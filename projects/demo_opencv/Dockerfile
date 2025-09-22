# 用公開的 base（對應 JetPack 6.0 DP / R36.2.0）
FROM nvcr.io/nvidia/l4t-base:r36.2.0

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /workspace

# 在「容器內」安裝你需要的東西（主機不會被影響）
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-opencv python3-numpy \
    gstreamer1.0-tools gstreamer1.0-libav \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    v4l-utils \
 && rm -rf /var/lib/apt/lists/*

COPY . .

CMD ["python3", "main.py"]
