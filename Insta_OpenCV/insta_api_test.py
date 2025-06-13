import cv2
import time
from Insta_OpenCV.controller.insta_worker import InstaWorker
from Insta_OpenCV.utils.frame_receiver import FrameReceiver


def main():
    print("[TEST] Init InstaWorker ...")
    worker = InstaWorker()
    ready_event = worker.start_live_all()
    print("[TEST] Waiting for worker to be ready...")
    ready_event.wait()
    print("[TEST] Worker ready! 等待推流穩定...")
    time.sleep(10)  # 等待推流穩定時間拉長到10秒

    cam_ids = [1, 5, 3]
    default_params = {'fx': 100, 'fy': 350, 'cx': 120, 'cy': 160, 'd0': 0.15, 'd1': 0.15}
    receiver = FrameReceiver(cam_ids, 240, 320, default_params)
    receiver.start()
    receiver.enable_trackbar('Cameras 1 5 3')
    cv2.namedWindow('Cameras 1 5 3', cv2.WINDOW_AUTOSIZE)  # 設為不可調整大小

    try:
        while True:
            receiver.update_params_from_trackbar()
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