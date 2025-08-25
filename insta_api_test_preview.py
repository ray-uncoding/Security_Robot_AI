import cv2
import time
from Insta_OpenCV.controller.insta_worker import InstaWorker
from Insta_OpenCV.utils.frame_receiver import FrameReceiver


def main():
    print("[TEST] Init InstaWorker ...")
    worker = InstaWorker()
    ready_event = worker.start_preview_all()  # 啟用 preview 模式
    print("[TEST] Waiting for worker to be ready...")
    ready_event.wait()
    print("[TEST] Worker ready! 等待推流穩定...")
    time.sleep(5)  # 等待推流穩定

    receiver = FrameReceiver()  # 預設只會啟動一個 process
    receiver.start()
    cv2.namedWindow('Insta Preview', cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            frame = receiver.get_latest_frame()  # 取得最新 preview 畫面
            if frame is not None:
                cv2.imshow('Insta Preview', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        receiver.stop()
        cv2.destroyAllWindows()
        worker.stop_all()
        print('[TEST] stop_all done!')

if __name__ == "__main__":
    main()
