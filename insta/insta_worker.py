from insta_manager import InstaManager
import threading

def insta_worker():
    """Insta360 Worker"""
    manager = InstaManager()
    if not manager.connect_camera():
        print("[InstaWorker] 無法連接相機，退出 Worker")
        return

    manager.start_polling()
    print("[InstaWorker] Insta360 Worker 已啟動")

    try:
        while True:
            # 在這裡可以加入其他 Insta360 的操作邏輯
            threading.Event().wait(1)
    except KeyboardInterrupt:
        print("[InstaWorker] 停止 Worker")
    finally:
        manager.disconnect_camera()

if __name__ == "__main__":
    # 單獨測試 Insta360 Worker
    print("[InstaWorker] 測試模式啟動")
    insta_worker()