#!/usr/bin/env python3
"""
測試 Gemini Live API 功能的簡單程式

測試內容：
1. 測試 Live API 連線
2. 測試音訊對話功能
3. 測試文字互動
4. 測試錯誤處理

使用方式：
1. 確保已設定 GOOGLE_API_KEY 或 GEMINI_API_KEY 環境變數
2. 執行 python test_live_api.py
3. 選擇測試模式
4. 按照提示進行互動
"""

import os
import sys
import time
import threading
from datetime import datetime
from typing import Optional, List

from gemini_client import GeminiClient
from shared_queue import log_queue_gemini

# Global variables for test state
test_client: Optional[GeminiClient] = None
received_texts: List[str] = []
received_audio_count: int = 0
test_start_time: Optional[float] = None

def print_separator() -> None:
    """打印分隔線。"""
    print("=" * 60)


def print_test_status(status: str) -> None:
    """打印測試狀態。
    
    Args:
        status: 要顯示的狀態訊息
    """
    timestamp = datetime.now().strftime("%H:%M:%S")
    print(f"[{timestamp}] {status}")


def on_text_received(text: str) -> None:
    """處理接收到的文字回應。
    
    Args:
        text: 接收到的文字內容
    """
    global received_texts
    received_texts.append(text)
    print_test_status(f"收到文字回應: {text}")


def on_audio_received(audio_data: bytes) -> None:
    """處理接收到的音訊回應。
    
    Args:
        audio_data: 接收到的音訊資料
    """
    global received_audio_count
    received_audio_count += 1
    print_test_status(f"收到音訊資料 (第 {received_audio_count} 筆, 大小: {len(audio_data)} bytes)")


def display_logs() -> None:
    """顯示日誌訊息。"""
    while not log_queue_gemini.empty():
        log_msg = log_queue_gemini.get()
        print(f"[LOG] {log_msg}")

def test_basic_connection() -> bool:
    """測試基本連線功能。
    
    Returns:
        bool: 測試成功返回 True，否則返回 False
    """
    print_separator()
    print("測試 1: 基本連線測試")
    print_separator()
    
    global test_client
    test_client = GeminiClient()
    
    # 顯示初始化日誌
    display_logs()
    
    # 檢查 API 金鑰
    if not test_client.api_key:
        print_test_status("❌ 錯誤: 未設定 API 金鑰")
        print("請設定環境變數 GOOGLE_API_KEY 或 GEMINI_API_KEY")
        return False
    
    print_test_status("✅ API 金鑰已設定")
    
    # 列出可用模型
    print_test_status("正在列出可用模型...")
    models = test_client.list_available_models()
    print(f"可用模型數量: {len(models)}")
    for model in models[:5]:  # 只顯示前5個
        print(f"  - {model}")
    
    display_logs()
    return True

def test_live_session() -> bool:
    """測試 Live API 會話。
    
    Returns:
        bool: 測試成功返回 True，否則返回 False
    """
    print_separator()
    print("測試 2: Live API 會話測試")
    print_separator()
    
    global test_client, received_texts, received_audio_count, test_start_time
    
    # 重置計數器
    received_texts = []
    received_audio_count = 0
    test_start_time = time.time()
    
    print_test_status("正在啟動 Live 會話...")
    print("模式: 純音訊對話（無視訊）")
    print("語音: Zephyr")
    
    # 啟動 Live 會話
    success = test_client.start_live_session(
        video_mode="none",
        voice_name="Zephyr",
        response_modalities=["AUDIO"],
        on_text_received=on_text_received,
        on_audio_received=on_audio_received
    )
    
    display_logs()
    
    if not success:
        print_test_status("❌ 無法啟動 Live 會話")
        return False
    
    print_test_status("✅ Live 會話已啟動")
    
    # 等待會話完全初始化
    print_test_status("等待會話初始化...")
    time.sleep(3)
    display_logs()
    
    # 測試發送文字訊息
    print_separator()
    print("測試 3: 發送文字訊息")
    print_separator()
    
    test_message = "你好，這是一個測試訊息。請用中文回應。"
    print_test_status(f"發送測試訊息: {test_message}")
    
    success = test_client.send_text_to_live(test_message)
    if success:
        print_test_status("✅ 文字訊息已發送")
    else:
        print_test_status("❌ 無法發送文字訊息")
    
    # 等待回應
    print_test_status("等待回應（10秒）...")
    for i in range(10):
        time.sleep(1)
        display_logs()
        if received_texts or received_audio_count > 0:
            break
    
    # 顯示測試結果
    print_separator()
    print("測試結果統計：")
    print(f"- 收到文字回應數: {len(received_texts)}")
    print(f"- 收到音訊資料數: {received_audio_count}")
    print(f"- 測試執行時間: {time.time() - test_start_time:.1f} 秒")
    
    if received_texts:
        print("\n收到的文字回應:")
        for i, text in enumerate(received_texts, 1):
            print(f"{i}. {text}")
    
    return True

def test_interactive_mode() -> bool:
    """互動測試模式。
    
    Returns:
        bool: 測試完成返回 True
    """
    print_separator()
    print("測試 4: 互動測試模式")
    print_separator()
    print("說明：")
    print("- 輸入文字後按 Enter 發送")
    print("- 輸入 'quit' 結束測試")
    print("- 系統會顯示收到的回應")
    print_separator()
    
    global test_client
    
    if not test_client.is_live_mode_active():
        print_test_status("請先執行測試 2 來啟動 Live 會話")
        return False
    
    while True:
        try:
            user_input = input("\n請輸入訊息 (或 'quit' 結束): ")
            
            if user_input.lower() == 'quit':
                break
            
            if not user_input.strip():
                continue
            
            # 發送訊息
            success = test_client.send_text_to_live(user_input)
            if success:
                print_test_status("✅ 訊息已發送")
            else:
                print_test_status("❌ 發送失敗")
            
            # 等待回應
            time.sleep(2)
            display_logs()
            
        except KeyboardInterrupt:
            print("\n中斷測試")
            break
    
    return True

def cleanup() -> None:
    """清理資源。
    
    停止 Live 會話並釋放相關資源。
    """
    print_separator()
    print("清理資源...")
    
    global test_client
    if test_client and test_client.is_live_mode_active():
        print_test_status("正在停止 Live 會話...")
        test_client.stop_live_session()
        time.sleep(1)
        display_logs()
        print_test_status("✅ Live 會話已停止")

def main() -> None:
    """主測試程式。
    
    執行完整的 Live API 測試流程。
    """
    print_separator()
    print("Gemini Live API 測試程式")
    print_separator()
    print(f"測試時間: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Python 版本: {sys.version.split()[0]}")
    print_separator()
    
    try:
        # 測試 1: 基本連線
        if not test_basic_connection():
            print("\n基本連線測試失敗，無法繼續")
            return
        
        print("\n按 Enter 繼續進行 Live API 測試...")
        input()
        
        # 測試 2: Live 會話
        if not test_live_session():
            print("\nLive 會話測試失敗")
            return
        
        # 詢問是否進行互動測試
        print("\n是否進行互動測試？(y/n): ", end="")
        choice = input().lower()
        
        if choice == 'y':
            test_interactive_mode()
        
    except Exception as e:
        print(f"\n發生錯誤: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # 清理資源
        cleanup()
        print("\n測試完成")

if __name__ == "__main__":
    main()