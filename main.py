from core import start_all_threads
from ui import launch_ui, update_camera_info, update_reid_results
from reid_manager import ReIDManager

def main():
    # 初始化模組
    reid_manager = ReIDManager()

    # 啟動其他系統模組
    start_all_threads()

    # 啟動 UI
    launch_ui()

if __name__ == "__main__":
    main()
