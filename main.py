from core.core import start_all_threads, stop_all_threads
from reid.reid_manager import ReIDManager
from PyQt5.QtWidgets import QApplication
from ui.ui_window import ControlPanel

def initialize_modules():
    """初始化系統模組"""
    print("[Main] Initializing modules...")
    reid_manager = ReIDManager()
    print("[Main] Modules initialized.")
    return reid_manager

def start_system():
    """啟動系統執行緒"""
    print("[Main] Starting all threads...")
    start_all_threads()
    print("[Main] All threads started.")

def stop_system():
    """停止系統執行緒"""
    print("[Main] Stopping all threads...")
    stop_all_threads()
    print("[Main] All threads stopped.")

def launch_ui():
    """啟動圖形化使用者介面"""
    print("[Main] Launching UI...")
    app = QApplication([])
    control_panel = ControlPanel()
    control_panel.show()
    app.exec_()

def main():
    try:
        # 初始化模組
        reid_manager = initialize_modules()

        # 啟動系統
        start_system()

        # 啟動 UI
        launch_ui()
    except Exception as error:
        print(f"[Main] Error: {error}")
    finally:
        # 確保系統在結束時正確清理
        stop_system()

if __name__ == "__main__":
    main()
