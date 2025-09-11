#!/usr/bin/env python3
"""測試 GPU 監控功能"""
import sys
import os
import time

# 添加專案路徑
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

def test_jtop_direct():
    """直接測試 jtop"""
    print("=== 測試 jtop 直接訪問 ===")
    try:
        from jtop import jtop
        with jtop() as jetson:
            if jetson.ok():
                print(f"jtop 連接成功")
                
                # GPU 統計
                gpu_stats = jetson.gpu
                print(f"GPU stats: {gpu_stats}")
                
                # 記憶體統計
                memory_stats = jetson.memory
                print(f"Memory stats: {memory_stats}")
                
                # 溫度統計
                temp_stats = jetson.temperature
                print(f"Temperature stats: {temp_stats}")
                
                return True
            else:
                print("jtop 連接失敗")
                return False
    except ImportError as e:
        print(f"jtop 模組導入失敗: {e}")
        return False
    except Exception as e:
        print(f"jtop 訪問錯誤: {e}")
        return False

def test_system_monitor():
    """測試 SystemMonitor 類別"""
    print("\n=== 測試 SystemMonitor 類別 ===")
    try:
        from Insta_OpenCV.services.system_monitor import SystemMonitor
        
        monitor = SystemMonitor(update_interval=1.0)
        monitor.start_monitoring()
        
        print("監控已啟動，等待5秒...")
        time.sleep(5)
        
        metrics = monitor.get_metrics()
        print(f"CPU: {metrics.cpu_percent:.1f}%")
        print(f"GPU: {metrics.gpu_percent:.1f}%")
        print(f"Memory: {metrics.memory_percent:.1f}%")
        print(f"GPU Memory: {metrics.gpu_memory_used_mb:.1f}MB / {metrics.gpu_memory_total_mb:.1f}MB")
        print(f"GPU Temp: {metrics.gpu_temp:.1f}°C")
        print(f"CPU Temp: {metrics.cpu_temp:.1f}°C")
        
        summary = monitor.get_resource_summary()
        print(f"\n資源摘要: {summary}")
        
        monitor.stop_monitoring()
        return True
        
    except Exception as e:
        print(f"SystemMonitor 測試失敗: {e}")
        return False

def test_gpu_load_generation():
    """測試 GPU 負載生成"""
    print("\n=== 測試 GPU 負載生成 ===")
    try:
        import cv2
        import numpy as np
        
        # 檢查 CUDA 可用性
        cuda_devices = cv2.cuda.getCudaEnabledDeviceCount()
        print(f"CUDA 設備數量: {cuda_devices}")
        
        if cuda_devices > 0:
            print("生成 GPU 負載...")
            
            # 創建一個大型影像
            large_image = np.random.randint(0, 255, (2000, 2000, 3), dtype=np.uint8)
            
            # 上傳到 GPU
            gpu_image = cv2.cuda_GpuMat()
            gpu_image.upload(large_image)
            
            # 執行一些 GPU 操作
            for i in range(100):
                gpu_blurred = cv2.cuda.bilateralFilter(gpu_image, -1, 5, 5)
                gpu_resized = cv2.cuda.resize(gpu_blurred, (1000, 1000))
                if i % 20 == 0:
                    print(f"GPU 操作進度: {i}/100")
            
            print("GPU 負載生成完成")
            return True
        else:
            print("沒有可用的 CUDA 設備")
            return False
            
    except Exception as e:
        print(f"GPU 負載生成失敗: {e}")
        return False

if __name__ == "__main__":
    print("GPU 監控測試腳本")
    print("=" * 50)
    
    # 測試 jtop 直接訪問
    jtop_ok = test_jtop_direct()
    
    # 測試 SystemMonitor
    monitor_ok = test_system_monitor()
    
    # 如果需要，生成 GPU 負載
    print("\n是否要生成 GPU 負載來測試監控？(y/N): ", end="")
    try:
        if input().lower() == 'y':
            test_gpu_load_generation()
            print("\n重新測試監控...")
            test_system_monitor()
    except KeyboardInterrupt:
        print("\n測試中斷")
    
    print("\n測試完成")
    print(f"jtop 直接訪問: {'✓' if jtop_ok else '✗'}")
    print(f"SystemMonitor: {'✓' if monitor_ok else '✗'}")
