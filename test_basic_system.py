#!/usr/bin/env python3
"""
基本系統測試腳本
測試不包含 Live 模式的基本功能
"""

import sys
import os
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

# 添加專案路徑
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from shared_queue import log_queue_system, stop_event
from core import start_all_threads, stop_all_threads
from gemini_client import GeminiClient

def test_basic_imports():
    """測試基本導入"""
    print("測試 1: 基本導入")
    try:
        from ui_window import ControlPanel
        from workers import gemini_worker
        print("✅ 所有基本導入成功")
        return True
    except Exception as e:
        print(f"❌ 導入失敗: {e}")
        return False

def test_gemini_client_basic():
    """測試 GeminiClient 基本功能"""
    print("\n測試 2: GeminiClient 基本功能")
    try:
        client = GeminiClient()
        print(f"✅ GeminiClient 初始化成功")
        
        # 測試模型列表
        models = client.list_available_models()
        print(f"✅ 可用模型: {models[:3]}..." if len(models) > 3 else f"✅ 可用模型: {models}")
        
        # 測試設定模型
        if client.set_model("gemini-1.5-flash"):
            print("✅ 模型設定成功")
        else:
            print("⚠️ 模型設定失敗，可能是 API 金鑰問題")
            
        return True
    except Exception as e:
        print(f"❌ GeminiClient 測試失敗: {e}")
        return False

def test_ui_basic():
    """測試 UI 基本功能"""
    print("\n測試 3: UI 基本啟動")
    try:
        app = QApplication(sys.argv)
        
        # 創建 UI 但不顯示
        from ui_window import ControlPanel
        window = ControlPanel()
        
        # 設定 5 秒後自動關閉
        timer = QTimer()
        timer.timeout.connect(app.quit)
        timer.start(5000)  # 5 秒
        
        print("✅ UI 初始化成功，將在 5 秒後關閉...")
        window.show()
        
        # 運行 5 秒後退出
        app.exec_()
        
        return True
    except Exception as e:
        print(f"❌ UI 測試失敗: {e}")
        return False

def test_workers_basic():
    """測試 Workers 基本功能"""
    print("\n測試 4: Workers 基本功能")
    try:
        print("啟動所有執行緒...")
        start_all_threads()
        
        import time
        time.sleep(3)  # 讓執行緒運行 3 秒
        
        print("停止所有執行緒...")
        stop_all_threads()
        
        print("✅ Workers 測試成功")
        return True
    except Exception as e:
        print(f"❌ Workers 測試失敗: {e}")
        return False

def main():
    """主測試函數"""
    print("=== Security Robot AI 基本系統測試 ===\n")
    
    tests = [
        test_basic_imports,
        test_gemini_client_basic,
        test_workers_basic,
        test_ui_basic,  # UI 測試放最後因為會啟動 Qt 應用
    ]
    
    results = []
    for test in tests:
        try:
            result = test()
            results.append(result)
        except Exception as e:
            print(f"❌ 測試異常: {e}")
            results.append(False)
    
    print(f"\n=== 測試結果 ===")
    print(f"通過: {sum(results)}/{len(results)} 項測試")
    
    if all(results):
        print("🎉 所有基本功能測試通過！")
        return 0
    else:
        print("⚠️ 部分測試失敗，請檢查相關模組")
        return 1

if __name__ == "__main__":
    sys.exit(main())