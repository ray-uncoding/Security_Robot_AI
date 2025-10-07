import cv2
import time
import subprocess
import numpy as np
import multiprocessing as mp
import threading
import psutil
import os
import json
import logging
from .gpu_processor import GPUProcessor
from ..services.performance_monitor import get_global_performance_monitor, FrameRateController

# 設定日誌
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def nothing(x):
    pass

def get_stream_mode():
    # 檢查 nginx live 流程是否存在，若有則為 live，否則為 preview
    for p in psutil.process_iter():
        try:
            if 'nginx.exe' in p.name().lower():
                return 'live'
        except Exception:
            continue
    return 'preview'

def get_preview_url():
    config_path = os.path.join(os.path.dirname(__file__), '../config/settings.json')
    with open(config_path, 'r', encoding='utf-8') as f:
        settings = json.load(f)
    return settings.get('rtmp_url')

def camera_process(cam_id, stream_url, width, height, frame_dict, param_dict):
    import psutil
    import time as _time
    import cv2 # Ensure cv2 is imported here if not already globally
    import numpy as np # Ensure np is imported here
    import os
    import ctypes.util

    # detect whether libnvcuvid is available in the container / host mounts
    has_cuvid = False
    # try find library via ctypes and common path
    if ctypes.util.find_library('nvcuvid'):
        has_cuvid = True
    elif os.path.exists('/usr/lib/aarch64-linux-gnu/libnvcuvid.so.1') or os.path.exists('/usr/lib/aarch64-linux-gnu/tegra/libnvcuvid.so.1'):
        has_cuvid = True

    stream_mode = get_stream_mode()
    if stream_mode == 'live':
        # keep existing ffmpeg-subprocess based pipeline for live/origin streams
        url = f'rtmp://192.168.1.188:1935/live/origin{cam_id}'
        if has_cuvid:
            decoder_args = ['-c:v', 'h264_cuvid']
            vf = 'scale_npp=w=320:h=240:format=bgr24:interp_algo=super'
        else:
            decoder_args = []
            vf = 'scale=w=320:h=240,format=bgr24'
        ffmpeg_cmd = [
            'ffmpeg',
        ] + decoder_args + [
            '-i', url,
            '-vf', vf,
            '-f', 'rawvideo',
            '-an', '-sn', '-dn',
            '-']
        target_shape = (240, 320, 3) # H, W, C for BGR24 output (w=320, h=240)
        do_undistort = True

        import subprocess # ensure subprocess is imported
        import queue # ensure queue is imported 
        def read_stderr(pipe, q):
            while True:
                line = pipe.stderr.readline()
                if not line:
                    break
                q.put(line.decode(errors='ignore'))
        pipe = subprocess.Popen(ffmpeg_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=10**8)
        stderr_q = queue.Queue()
        t = threading.Thread(target=read_stderr, args=(pipe, stderr_q), daemon=True)
        t.start()
        ffmpeg_proc = psutil.Process(pipe.pid)
        last_log = _time.time()
        try:
            while True:
                # Expected size for BGR24 frame
                expected_frame_size = target_shape[0] * target_shape[1] * 3
                raw_frame = pipe.stdout.read(expected_frame_size)
                if not raw_frame or len(raw_frame) != expected_frame_size:
                    print(f"[cam{cam_id}] Incomplete frame data or pipe closed. Expected {expected_frame_size}, got {len(raw_frame) if raw_frame else 0}. Check ffmpeg stderr.")
                    # Check stderr queue for ffmpeg errors
                    while not stderr_q.empty():
                        print(f"[ffmpeg-stderr][cam{cam_id}] {stderr_q.get_nowait()}")
                    _time.sleep(0.1) # Avoid busy-looping if pipe is broken
                    continue
                frame = np.frombuffer(raw_frame, np.uint8).reshape(target_shape)

                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

                if do_undistort:
                    params = param_dict['params']
                    K = np.array([[params['fx'], 0, params['cx']], [0, params['fy'], params['cy']], [0, 0, 1]], dtype=np.float32)
                    D = np.array([params['d0'], params['d1'], 0, 0], dtype=np.float32)
                    current_frame_shape = frame.shape
                    DIM = (current_frame_shape[1], current_frame_shape[0])
                    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
                    frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
                
                frame_dict[cam_id] = frame
                now = _time.time()
                if now - last_log > 2:
                    try:
                        cpu = ffmpeg_proc.cpu_percent(interval=0.1)
                        mem = ffmpeg_proc.memory_info().rss / 1024 / 1024
                        print(f"[ffmpeg][cam{cam_id}] CPU: {cpu:.1f}%  RAM: {mem:.1f}MB  PID: {pipe.pid}")
                    except Exception as e:
                        print(f"[ffmpeg][cam{cam_id}] 資源監控失敗: {e}")
                    last_log = now
        except KeyboardInterrupt:
            pass
        finally:
            try:
                pipe.terminate()
            except Exception:
                pass

    else: # preview mode - use OpenCV VideoCapture (no subprocess)
        url = stream_url if stream_url else get_preview_url()
        do_undistort = False

        cap = None
        reconnect_delay = 1.0
        last_log = _time.time()
        try:
            while True:
                if cap is None:
                    # try open with ffmpeg backend if available, else default
                    try:
                        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
                        if not cap.isOpened():
                            cap.release()
                            cap = cv2.VideoCapture(url)
                    except Exception:
                        cap = cv2.VideoCapture(url)
                        
                    # 設定低延遲屬性
                    if cap.isOpened():
                        cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)  # 設定緩衝區大小為0，最低延遲
                        cap.set(cv2.CAP_PROP_FPS, 30)  # 回到30 FPS
                        # 設定更多低延遲屬性
                        try:
                            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('H', '2', '6', '4'))
                            # 強制設定較小解析度以減少處理時間
                            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
                            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1920)
                        except:
                            pass
                        print(f"[cam{cam_id}] VideoCapture opened with ultra-low-latency settings")
                        
                    if not cap.isOpened():
                        print(f"[cam{cam_id}] VideoCapture open failed, retrying in {reconnect_delay}s")
                        try:
                            cap.release()
                        except Exception:
                            pass
                        cap = None
                        _time.sleep(reconnect_delay)
                        continue

                ret, frame = cap.read()
                if not ret or frame is None:
                    print(f"[cam{cam_id}] VideoCapture read failed, reconnecting...")
                    try:
                        cap.release()
                    except Exception:
                        pass
                    cap = None
                    _time.sleep(reconnect_delay)
                    continue

                # 激進的緩衝區清理 - 直接跳過所有積壓的幀
                skip_count = 0
                while cap.get(cv2.CAP_PROP_BUFFERSIZE) > 0 and skip_count < 10:
                    ret_temp, frame_temp = cap.read()
                    if ret_temp and frame_temp is not None:
                        frame = frame_temp
                        skip_count += 1
                    else:
                        break

                frame_dict[cam_id] = frame

                now = _time.time()
                if now - last_log > 2:
                    try:
                        print(f"[cap][cam{cam_id}] frame size {frame.shape}")
                    except Exception:
                        pass
                    last_log = now

                # 移除睡眠以提高響應速度
                # _time.sleep(0.01)
        except KeyboardInterrupt:
            pass
        finally:
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass

class FrameReceiver:
    def __init__(self, stream_url=None, cam_ids=None, width=240, height=320, params=None):
        self.stream_url = stream_url
        # 初始化 GPU 處理器
        self.gpu_processor = GPUProcessor()
        logger.info(f"[FrameReceiver] GPU available: {self.gpu_processor.is_gpu_available()}")
        
        # 初始化性能監控
        self.performance_monitor = get_global_performance_monitor()
        self.frame_rate_controller = FrameRateController(target_fps=30.0)
        
        # If a specific stream_url is provided, force single-camera preview mode.
        if self.stream_url:
            cam_ids = [0]
        else:
            stream_mode = get_stream_mode()
            if stream_mode == 'preview':
                cam_ids = [0]  # 只啟動一個 process
            elif cam_ids is None:
                # 修改為 1, 5, 3, 2, 6, 4
                cam_ids = [1, 5, 3, 2, 6, 4]

        if isinstance(cam_ids, int):
            cam_ids = [cam_ids]
        self.cam_ids = cam_ids
        self.width = width
        self.height = height
        self._manager = mp.Manager()
        self._frame_dict = self._manager.dict()
        self._param_dict = self._manager.dict()
        if params is None:
            params = {'fx': 100, 'fy': 350, 'cx': 120, 'cy': 160, 'd0': 0.15, 'd1': 0.15}
        self._param_dict['params'] = params.copy()
        self._processes = []
        self._running = False
        self._use_trackbar = False

    def start(self):
        if self._running:
            return
        for cam_id in self.cam_ids:
            # Pass the stream_url to the target process
            p = mp.Process(target=camera_process, args=(cam_id, self.stream_url, self.width, self.height, self._frame_dict, self._param_dict))
            p.start()
            self._processes.append(p)
        self._running = True
        
        # 啟動性能監控
        self.performance_monitor.start_monitoring()
        logger.info("[FrameReceiver] Started with performance monitoring")

    def stop(self):
        for p in self._processes:
            p.terminate()
            p.join()
        self._processes = []
        self._running = False
        
        # 停止性能監控
        self.performance_monitor.stop_monitoring()
        logger.info("[FrameReceiver] Stopped")

    def get_latest_frame(self):
        """獲取最新影像幀"""
        frame = None
        if len(self.cam_ids) == 1:
            frame = self._frame_dict.get(self.cam_ids[0], None)
        else:
            raise RuntimeError('Use get_grid_frame() for multi-cam mode')
        
        # 記錄性能指標
        if frame is not None:
            self.performance_monitor.record_frame()
        
        return frame
    
    def get_latest_frame_processed(self, target_width=None, target_height=None, enhance_quality=False):
        """
        獲取 GPU 處理後的最新影像幀
        
        Args:
            target_width: 目標寬度（可選）
            target_height: 目標高度（可選）
            enhance_quality: 是否進行品質增強
            
        Returns:
            處理後的影像幀
        """
        start_time = time.time()
        
        frame = self.get_latest_frame()
        if frame is None:
            return None
        
        # 品質增強
        if enhance_quality:
            frame = self.gpu_processor.enhance_frame_quality(frame)
        
        # 縮放處理
        if target_width and target_height:
            frame = self.gpu_processor.resize_frame(frame, target_width, target_height)
        
        # 記錄處理時間
        processing_time = time.time() - start_time
        self.performance_monitor.record_processing_time(processing_time)
        
        return frame
    
    def get_roi_from_frame(self, x, y, width, height, enhance_quality=False):
        """
        從最新幀中提取 ROI 區域（GPU 加速）
        
        Args:
            x, y: ROI 起始座標
            width, height: ROI 尺寸
            enhance_quality: 是否進行品質增強
            
        Returns:
            ROI 影像
        """
        frame = self.get_latest_frame()
        if frame is None:
            return None
        
        # 確保座標在有效範圍內
        h, w = frame.shape[:2]
        x = max(0, min(x, w - 1))
        y = max(0, min(y, h - 1))
        width = min(width, w - x)
        height = min(height, h - y)
        
        if width <= 0 or height <= 0:
            return None
        
        # GPU 加速裁剪
        roi_frame = self.gpu_processor.crop_frame(frame, x, y, width, height)
        
        # 品質增強
        if enhance_quality:
            roi_frame = self.gpu_processor.enhance_frame_quality(roi_frame)
        
        return roi_frame

    def get_grid_frame(self):
        stream_mode = get_stream_mode()
        if stream_mode == 'preview':
            frame = self._frame_dict.get(0, None)
            if frame is None or frame.shape[0] != 320 or frame.shape[1] != 640:
                frame = np.zeros((320, 640, 3), dtype=np.uint8)
            return frame
        # live 模式下拼接 1,5,3 和 2,6,4
        # 攝影機畫面在 camera_process 中會被逆時針旋轉90度，所以預期 shape 為 (320, 240, 3)
        id_grid = [[1, 5, 3], [2, 6, 4]] # 修改網格佈局
        expected_shape = (320, 240, 3) # H, W, C after rotation
        
        all_row_frames = []
        for row_ids in id_grid:
            frames_in_row = []
            for cam_id in row_ids:
                frame = self._frame_dict.get(cam_id, None)
                # 檢查獲取的幀是否有效且形狀正確
                if frame is None or frame.shape != expected_shape:
                    # 如果無效，則使用黑色畫面填充
                    frame = np.zeros(expected_shape, dtype=np.uint8)
                frames_in_row.append(frame)
            # 水平拼接單行的所有幀
            all_row_frames.append(np.hstack(frames_in_row))
        
        # 垂直拼接所有行
        if not all_row_frames: # 如果沒有任何行（例如 cam_ids 為空）
            # 返回一個與預期單幀相同大小的黑色畫面，或一個預設的網格大小
            # 這裡我們假設如果 id_grid 有定義，至少會嘗試創建行
            # 如果 cam_ids 為空導致 id_grid 也為空，則需要一個更通用的回退
            return np.zeros((expected_shape[0], expected_shape[1] * 3, 3), dtype=np.uint8) # 預設單行3個畫面大小

        return np.vstack(all_row_frames)

    def get_params(self):
        return self._param_dict['params'].copy()

    def set_params(self, params):
        self._param_dict['params'] = params.copy()

    def is_running(self):
        return self._running

    def enable_trackbar(self, window_name='Camera'):
        self._trackbar_window = window_name
        params = self.get_params()
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.createTrackbar('fx', window_name, int(params['fx'])-100, 400, nothing)
        cv2.createTrackbar('fy', window_name, int(params['fy'])-100, 400, nothing)
        cv2.createTrackbar('cx', window_name, int(params['cx']), 320, nothing)
        cv2.createTrackbar('cy', window_name, int(params['cy']), 240, nothing)
        cv2.createTrackbar('d0', window_name, int(params['d0']*200)+100, 200, nothing)
        cv2.createTrackbar('d1', window_name, int(params['d1']*200)+100, 200, nothing)
        self._use_trackbar = True

    def update_params_from_trackbar(self):
        if not self._use_trackbar:
            return
        wn = self._trackbar_window
        fx = cv2.getTrackbarPos('fx', wn) + 100
        fy = cv2.getTrackbarPos('fy', wn) + 100
        cx = cv2.getTrackbarPos('cx', wn)
        cy = cv2.getTrackbarPos('cy', wn)
        d0 = (cv2.getTrackbarPos('d0', wn) - 100) / 200.0
        d1 = (cv2.getTrackbarPos('d1', wn) - 100) / 200.0
        params = {'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy, 'd0': d0, 'd1': d1}
        self.set_params(params)
    
    def get_gpu_info(self):
        """獲取 GPU 處理器資訊"""
        return self.gpu_processor.get_gpu_info()
    
    def is_gpu_available(self):
        """檢查 GPU 是否可用"""
        return self.gpu_processor.is_gpu_available()
    
    def get_performance_summary(self):
        """獲取性能摘要"""
        return self.performance_monitor.get_performance_summary()
    
    def get_performance_metrics(self):
        """獲取詳細性能指標"""
        return self.performance_monitor.get_metrics()
    
    def set_target_fps(self, fps: float):
        """設定目標幀率"""
        self.frame_rate_controller.set_target_fps(fps)
