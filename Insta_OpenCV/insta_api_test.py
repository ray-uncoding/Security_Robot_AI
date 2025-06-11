import cv2
import time
from Insta_OpenCV.controller.insta_worker import InstaWorker

def main():
    print("[TEST] Init InstaWorker ...")
    worker = InstaWorker()
    ready_event = worker.start_all()
    print("[TEST] Waiting for worker to be ready...")
    ready_event.wait()  # 等待 RTMP/心跳/FrameReceiver 都 ready
    print("[TEST] Worker ready! Start OpenCV preview...")
    while True:
        frame = worker.get_latest_frame()
        if frame is not None:
            max_width = 1280
            if frame.shape[1] > max_width:
                scale = max_width / frame.shape[1]
                frame = cv2.resize(frame, (max_width, int(frame.shape[0] * scale)))
            cv2.imshow('RTMP Preview', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            print("[TEST] No frame yet, waiting...")
            time.sleep(0.1)
    cv2.destroyAllWindows()
    print("[TEST] RTMP preview ended, stopping all...")
    worker.stop_all()
    print("[TEST] stop_all done!")

if __name__ == "__main__":
    main()
