import sys
import os
# 把專案根加入搜尋路徑（使 Insta_OpenCV 可被 import）
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

import cv2
import time
from Insta_OpenCV.controller.insta_worker import InstaWorker
from Insta_OpenCV.utils.frame_receiver import FrameReceiver

# Qt5 imports
try:
    from PyQt5 import QtWidgets, QtGui, QtCore
except ImportError:
    raise RuntimeError("需要安裝 PyQt5：/usr/bin/python -m pip install PyQt5")


class PreviewLabel(QtWidgets.QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setMinimumSize(640, 480)

    @QtCore.pyqtSlot(object)
    def update_frame(self, frame):
        if frame is None:
            return
        # OpenCV BGR -> RGB
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        qt_image = QtGui.QImage(rgb.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap.fromImage(qt_image)
        self.setPixmap(pix.scaled(self.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))


def main():
    # 初始化 worker 與 preview
    print("[TEST] Init InstaWorker ...")
    worker = InstaWorker()
    ready_event = worker.start_preview_all()
    print("[TEST] Waiting for worker to be ready...")
    ready_event.wait()
    print("[TEST] Worker ready! 等待推流穩定...")
    time.sleep(2)

    receiver = FrameReceiver()
    receiver.start()

    app = QtWidgets.QApplication(sys.argv)
    win = QtWidgets.QMainWindow()
    label = PreviewLabel()
    win.setCentralWidget(label)
    win.setWindowTitle("Insta Preview (Qt5)")
    win.resize(800, 600)

    # 使用 QTimer 週期抓取最新 frame 並更新 UI（非阻塞）
    timer = QtCore.QTimer()
    timer.setInterval(30)  # 大約 33ms -> ~30 FPS
    def on_timer():
        frame = receiver.get_latest_frame()
        if frame is not None:
            label.update_frame(frame)
    timer.timeout.connect(on_timer)
    timer.start()

    # 在視窗關閉時清理
    def on_about_to_quit():
        timer.stop()
        receiver.stop()
        worker.stop_all()
        print("[TEST] stop_all done!")
    app.aboutToQuit.connect(on_about_to_quit)

    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
