#!/usr/bin/env python3
"""Gemini Live API connection diagnostic script.

This script provides comprehensive diagnostics for Live API connection issues,
testing various configurations and analyzing error patterns.
"""

import os
import asyncio
import time
import traceback
from typing import List, Dict, Any, Optional

from gemini_client import GeminiClient
from shared_queue import log_queue_gemini

def collect_logs(timeout: int = 10) -> List[str]:
    """收集指定時間內的所有日誌。
    
    Args:
        timeout: 收集日誌的時間（秒）
        
    Returns:
        List[str]: 收集到的日誌訊息列表
    """
    logs: List[str] = []
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        while not log_queue_gemini.empty():
            try:
                log = log_queue_gemini.get_nowait()
                logs.append(log)
                print(log)
            except:
                break
        time.sleep(0.1)
    
    return logs

def print_environment_info() -> None:
    """打印環境資訊。
    
    顯示 Python 版本、API Key 設定狀態和必要套件的安裝情況。
    """
    print("=== 環境資訊 ===")
    print(f"Python 版本: {os.sys.version}")
    print(f"API Key 設定: {'已設定' if os.getenv('GOOGLE_API_KEY') or os.getenv('GEMINI_API_KEY') else '未設定'}")
    
    # 檢查必要的套件
    try:
        import google.genai
        print(f"google.genai 版本: {google.genai.__version__ if hasattr(google.genai, '__version__') else '已安裝'}")
    except ImportError as e:
        print(f"google.genai 導入錯誤: {e}")
    
    try:
        import pyaudio
        print("pyaudio: 已安裝")
    except ImportError as e:
        print(f"pyaudio 導入錯誤: {e}")
    
    print()

def test_basic_client() -> Optional[GeminiClient]:
    """測試基本客戶端功能。
    
    Returns:
        Optional[GeminiClient]: 成功則返回客戶端實例，失敗則返回 None
    """
    print("=== 測試 1: 基本客戶端功能 ===")
    
    try:
        client = GeminiClient()
        collect_logs(2)  # 收集初始化日誌
        
        print("\n--- 測試模型列表 ---")
        models = client.list_available_models()
        print(f"可用模型: {models}")
        collect_logs(2)
        
        print("\n--- 測試基本文字生成 ---")
        response = client.generate_response("測試訊息")
        print(f"回應: {response[:100]}..." if len(str(response)) > 100 else f"回應: {response}")
        collect_logs(2)
        
        return client
        
    except Exception as e:
        print(f"基本客戶端測試失敗: {e}")
        traceback.print_exc()
        collect_logs(2)
        return None

def test_live_connection_step_by_step(client: Optional[GeminiClient]) -> bool:
    """逐步測試 Live 連接。
    
    Args:
        client: GeminiClient 實例
        
    Returns:
        bool: 如果任何配置成功則返回 True，否則返回 False
    """
    print("\n=== 測試 2: Live 連接逐步診斷 ===")
    
    if not client:
        print("無法進行 Live 測試：基本客戶端未初始化")
        return False
    
    # 測試不同的配置組合
    test_configs = [
        {
            'name': '基本音訊模式',
            'video_mode': 'none',
            'voice_name': 'Zephyr',
            'response_modalities': ['AUDIO']
        },
        {
            'name': '文字+音訊模式',
            'video_mode': 'none', 
            'voice_name': 'Zephyr',
            'response_modalities': ['AUDIO', 'TEXT']
        },
        {
            'name': '不同語音',
            'video_mode': 'none',
            'voice_name': 'Coral',
            'response_modalities': ['AUDIO']
        }
    ]
    
    for i, config in enumerate(test_configs, 1):
        print(f"\n--- 測試 2.{i}: {config['name']} ---")
        print(f"配置: {config}")
        
        try:
            success = client.start_live_session(
                video_mode=config['video_mode'],
                voice_name=config['voice_name'],
                response_modalities=config['response_modalities']
            )
            
            print(f"啟動結果: {success}")
            
            # 收集啟動過程中的日誌
            logs = collect_logs(5)
            
            if success:
                print("Live 模式啟動成功！等待 3 秒後停止...")
                time.sleep(3)
                
                stop_success = client.stop_live_session()
                print(f"停止結果: {stop_success}")
                collect_logs(2)
                
                return True  # 如果任何一個配置成功，就返回成功
            else:
                print("Live 模式啟動失敗")
                
        except Exception as e:
            print(f"Live 連接測試發生錯誤: {e}")
            traceback.print_exc()
            collect_logs(2)
        
        print("-" * 50)
    
    return False

def analyze_error_patterns(logs: List[str]) -> Dict[str, str]:
    """分析錯誤模式。
    
    Args:
        logs: 日誌訊息列表
        
    Returns:
        Dict[str, str]: 發現的錯誤模式和描述
    """
    print("\n=== 錯誤分析 ===")
    
    error_patterns = {
        '1007': 'WebSocket 1007 錯誤 - 無效的幀載荷數據',
        'invalid argument': 'API 參數無效',
        'authentication': '認證問題',
        'permission': '權限問題',
        'model': '模型相關問題',
        'config': '配置問題',
        'timeout': '連接超時',
        'network': '網路問題'
    }
    
    found_patterns = {}
    
    # 收集所有日誌進行分析
    all_logs = []
    for log in logs:
        all_logs.append(str(log).lower())
    
    log_text = ' '.join(all_logs)
    
    for pattern, description in error_patterns.items():
        if pattern in log_text:
            found_patterns[pattern] = description
    
    if found_patterns:
        print("發現的錯誤模式:")
        for pattern, description in found_patterns.items():
            print(f"  - {pattern}: {description}")
    else:
        print("未發現特定的錯誤模式")
    
    return found_patterns

def main() -> None:
    """主診斷函數。
    
    執行完整的 Live API 連接診斷流程。
    """
    print("開始 Gemini Live API 連接問題診斷")
    print("=" * 60)
    
    # 環境資訊
    print_environment_info()
    
    # 收集所有日誌
    all_logs = []
    
    # 測試基本功能
    client = test_basic_client()
    
    # 測試 Live 連接
    live_success = False
    if client:
        live_success = test_live_connection_step_by_step(client)
    
    # 最終收集日誌
    final_logs = collect_logs(2)
    all_logs.extend(final_logs)
    
    # 分析錯誤
    error_patterns = analyze_error_patterns(all_logs)
    
    # 總結
    print("\n" + "=" * 60)
    print("診斷總結:")
    print(f"✓ 基本客戶端: {'成功' if client else '失敗'}")
    print(f"✗ Live 模式: {'成功' if live_success else '失敗'}")
    
    if not live_success:
        print("\n建議的修復步驟:")
        
        if '1007' in error_patterns:
            print("1. 檢查模型名稱是否正確")
            print("2. 驗證 LiveConnectConfig 參數")
            print("3. 確認 API 權限包含 Live API")
        
        if 'model' in error_patterns:
            print("1. 使用正確的 Live 模型名稱")
            print("2. 檢查模型是否對您的帳戶可用")
        
        if 'config' in error_patterns:
            print("1. 簡化 LiveConnectConfig 設定")
            print("2. 移除可能有問題的進階參數")
        
        print("4. 檢查網路連接和防火牆設定")
        print("5. 更新 google.genai 套件到最新版本")

if __name__ == "__main__":
    main()