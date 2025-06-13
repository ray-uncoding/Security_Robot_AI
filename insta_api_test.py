import cv2
import time
from Insta_OpenCV.controller.insta_worker import InstaWorker
from Insta_OpenCV.utils.frame_receiver import FrameReceiver


def main():
    print("[TEST] Init InstaWorker ...")
    worker = InstaWorker()
    # 改為啟用 live 模式
    ready_event = worker.start_live_all()  # 啟用 live 模式
    print("[TEST] Waiting for worker to be ready...")
    ready_event.wait()
    print("[TEST] Worker ready! 等待推流穩定...")
    time.sleep(10)  # 等待推流穩定時間拉長到10秒

    receiver = FrameReceiver()  # 自動偵測模式、自動選擇預設鏡頭與拉流路徑
    receiver.start()
    cv2.namedWindow('Cameras 1 5 3', cv2.WINDOW_AUTOSIZE)  # 設為不可調整大小

    try:
        while True:
            grid = receiver.get_grid_frame()
            cv2.imshow('Cameras 1 5 3', grid)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        receiver.stop()
        cv2.destroyAllWindows()
        worker.stop_all()
        print('[TEST] stop_all done!')

if __name__ == "__main__":
    main()