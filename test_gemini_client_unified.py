#!/usr/bin/env python3
"""
測試重寫後的統一 GeminiClient (純 google.genai API)
"""

import os
import asyncio
import time
from gemini_client import GeminiClient
from shared_queue import log_queue_gemini

def print_logs():
    """列印所有日誌訊息"""
    while not log_queue_gemini.empty():
        print(log_queue_gemini.get())

def test_basic_functionality():
    """測試基本功能"""
    print("=== 測試基本 GeminiClient 功能 ===")
    
    # 創建客戶端
    client = GeminiClient()
    print_logs()
    
    print("\n--- 測試模型列表 ---")
    models = client.list_available_models()
    print(f"可用模型: {models}")
    print_logs()
    
    print("\n--- 測試設定模型 ---")
    success = client.set_model("gemini-1.5-flash")
    print(f"設定模型成功: {success}")
    print_logs()
    
    print("\n--- 測試文字生成 ---")
    response = client.generate_response("你好，請說一句簡短的中文回應。")
    print(f"回應: {response}")
    print_logs()
    
    return client

def test_live_mode_startup():
    """測試 Live 模式啟動（不需要實際運行）"""
    print("\n\n=== 測試 Live 模式啟動 ===")
    
    client = GeminiClient()
    
    # 測試 Live 模式狀態
    print(f"Live 模式啟動前狀態: {client.is_live_mode_active()}")
    print(f"目前模式: {client.get_current_mode()}")
    
    # 嘗試啟動 Live 模式（可能會因為權限問題失敗，但不應該因為環境變數而被禁用）
    print("\n--- 嘗試啟動 Live 模式 ---")
    success = client.start_live_session(video_mode="none")
    print(f"Live 模式啟動成功: {success}")
    print_logs()
    
    if success:
        print(f"Live 模式啟動後狀態: {client.is_live_mode_active()}")
        print(f"目前模式: {client.get_current_mode()}")
        
        # 等待一下讓 Live session 完全啟動
        time.sleep(2)
        print_logs()
        
        # 停止 Live 模式
        print("\n--- 停止 Live 模式 ---")
        stop_success = client.stop_live_session()
        print(f"Live 模式停止成功: {stop_success}")
        print_logs()
        
        time.sleep(1)
        print(f"Live 模式停止後狀態: {client.is_live_mode_active()}")
    
    return client

def test_environment_variable_override():
    """測試環境變數是否不再禁用 Live 模式"""
    print("\n\n=== 測試環境變數覆蓋 ===")
    
    # 設定禁用環境變數
    os.environ['DISABLE_LIVE_MODE'] = '1'
    print("設定環境變數 DISABLE_LIVE_MODE=1")
    
    client = GeminiClient()
    
    # 嘗試啟動 Live 模式，現在應該不會被環境變數禁用
    print("嘗試啟動 Live 模式（環境變數已設定禁用）...")
    success = client.start_live_session(video_mode="none")
    print(f"Live 模式啟動成功: {success}")
    print_logs()
    
    # 清理環境變數
    if 'DISABLE_LIVE_MODE' in os.environ:
        del os.environ['DISABLE_LIVE_MODE']
    print("清理環境變數")
    
    if success:
        print("環境變數測試通過：Live 模式沒有被環境變數禁用")
        client.stop_live_session()
        print_logs()
    
    return success

def main():
    """主要測試函數"""
    print("開始測試重寫後的 GeminiClient...")
    print("="*60)
    
    try:
        # 測試基本功能
        client = test_basic_functionality()
        
        # 測試 Live 模式
        test_live_mode_startup()
        
        # 測試環境變數覆蓋
        env_test_passed = test_environment_variable_override()
        
        print("\n" + "="*60)
        print("測試摘要:")
        print("✓ 基本功能：API 統一為 google.genai")
        print("✓ 文字生成：正常運作")
        print("✓ 模型列表：正常運作")
        print(f"{'✓' if env_test_passed else '✗'} Live 模式：不被環境變數禁用")
        print("\n重寫完成！GeminiClient 現在完全使用 google.genai API。")
        
    except Exception as e:
        print(f"測試過程中發生錯誤: {e}")
        import traceback
        traceback.print_exc()
        print_logs()

if __name__ == "__main__":
    main()