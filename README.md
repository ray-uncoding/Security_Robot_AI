# 🔐 Security Robot AI

本專案為一套模組化、可擴充的保全機器人 AI 系統，具備多執行緒架構、即時人臉辨識、ReID 身份比對，並整合 PyQt 圖形化使用者介面。

---

## 📂 專案目錄架構

```bash
Security_Robot_AI/
├── main.py               # 主程式入口，負責啟動執行緒與 GUI
├── core.py               # 執行緒集中管理模組
├── ui.py                 # 啟動 PyQt5 應用的控制模組
├── ui_window.py          # UI 顯示邏輯與控制（ControlPanel 類別）
├── workers.py            # 背景執行緒模組：RTMP、FaceDetector、ReID
├── shared_queue.py       # 所有模組共享的 queue 與 stop_event 定義
├── README.md             # 本說明文件
```

---

## ♻️ 模組說明

| 模組檔案         | 功能簡述 |
|------------------|----------|
| `main.py`        | 系統入口，呼叫核心與 UI 模組 |
| `core.py`        | 建立並啟動所有背景執行緒 |
| `workers.py`     | 背景任務模組：
- `rtmp_worker` 推送模擬影像
- `face_detector_worker` 模擬人臉擷取與特徵產生
- `reid_worker` 身分辨識模擬
| `shared_queue.py`| 定義模組共用的佇列與停止旗標 |
| `ui_window.py`   | PyQt UI 顯示邏輯（QGroupBox 分區） |
| `ui.py`          | 啟動 UI 主介面應用 |

---

## 📆 依賴安裝

```bash
pip install PyQt5
```

---

## ⚡️ 執行方式

```bash
python main.py
```

---

## 🚀 發展方向（建議）
- [ ] 整合 OpenCV 串流顯示
- [ ] 串接 YOLOv8 或人臉辨識模型
- [ ] 將識別記錄寫入 SQLite/CSV
- [ ] 黑名單比對與警報機制
- [ ] 將 UI 轉為多頁/分頁系統

---

Made with ❤️ for secure, modular, and scalable robot AI systems.