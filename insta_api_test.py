import cv2
import time
import subprocess
import numpy as np
from Insta_OpenCV.controller.insta_worker import InstaWorker

def main():
    print("[TEST] Init InstaWorker ...")
    worker = InstaWorker()
    ready_event = worker.start_live_all()  # 啟動 live 串流
    print("[TEST] Waiting for worker to be ready...")
    ready_event.wait()  # 等待 RTMP/心跳/FrameReceiver 都 ready
    print("[TEST] Worker ready! 等待推流穩定...")
    time.sleep(3)  # 可視情況調整，確保推流端已經穩定推流

    # 多路顯示 1、3、5 號魚眼鏡頭
    cam_ids = [1, 3, 5]
    pipes = []
    for cam_id in cam_ids:
        ffmpeg_cmd = [
            r'./ffmpeg-master-latest-win64-gpl-shared/bin/ffmpeg.exe',
            '-fflags', 'nobuffer',
            '-flags', 'low_delay',
            '-threads', '2',
            '-analyzeduration', '10000000',
            '-probesize', '10000000',
            '-i', f'rtmp://192.168.1.188:1935/live/origin{cam_id}',
            '-vf', 'scale=480:-1,transpose=2',  # 再降解析度到 480 寬
            '-r', '15',  # 降低幀率到 15fps
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-an', '-sn', '-dn',
            '-']
        width, height = int(2880 * 480 / 3840), 480  # 旋轉後寬高對調
        pipe = subprocess.Popen(ffmpeg_cmd, stdout=subprocess.PIPE, bufsize=10**8)
        pipes.append((pipe, width, height, f'RTMP origin{cam_id}'))
    print('[TEST] ffmpeg pipes started, OpenCV preview...')
    while True:
        for pipe, width, height, win_name in pipes:
            raw_frame = pipe.stdout.read(width * height * 3)
            if len(raw_frame) != width * height * 3:
                print(f'[TEST] No frame or broken pipe for {win_name}, waiting...')
                continue
            frame = np.frombuffer(raw_frame, np.uint8).reshape((height, width, 3))
            cv2.imshow(win_name, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
    for pipe, _, _, _ in pipes:
        pipe.terminate()
    print('[TEST] ffmpeg pipes ended.')
    worker.stop_all()
    print('[TEST] stop_all done!')

if __name__ == "__main__":
    main()