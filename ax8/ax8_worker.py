from ax8_manager import AX8Manager
import threading

def ax8_worker(url, username, password):
    """
    AX8 Worker
    :param url: AX8 相機的快照 URL
    :param username: 登錄用戶名
    :param password: 登錄密碼
    """
    manager = AX8Manager(url, username, password)
    try:
        if manager.login():
            print("[AX8Worker] 登錄成功，開始顯示影像串流")
            manager.display_stream()
        else:
            print("[AX8Worker] 登錄失敗，無法顯示影像串流")
    except KeyboardInterrupt:
        print("[AX8Worker] 停止 Worker")
    finally:
        manager.stop()

if __name__ == "__main__":
    AX8_URL = "http://192.168.1.100/snapshot.jpg"
    USERNAME = "admin"
    PASSWORD = "admin"

    # 啟動 AX8 Worker
    ax8_worker(AX8_URL, USERNAME, PASSWORD)