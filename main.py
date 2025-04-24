from core.core import start_all_threads, stop_all_threads
from reid.reid_manager import ReIDManager
from PyQt5.QtWidgets import QApplication
from ui.ui_window import ControlPanel

def main():
    # 初始化模組
    reid_manager = ReIDManager()

    # 啟動其他系統模組
    start_all_threads()

    # 啟動 UI
    app = QApplication([])
    window = ControlPanel()
    window.show()
    app.exec_()

if __name__ == "__main__":
    main()
