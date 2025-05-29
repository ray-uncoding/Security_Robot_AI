#!/usr/bin/env python3
"""
GeminiClient 統一介面使用範例

展示如何使用新的統一 GeminiClient 類別，
支援傳統文字模式和 Live 互動模式
"""

import asyncio
import time
from gemini_client import GeminiClient

def demo_text_mode():
    """演示傳統文字模式（保持向後相容性）"""
    print("=== 傳統文字模式演示 ===")
    
    # 初始化客戶端
    client = GeminiClient()
    
    # 列出可用模型
    models = client.list_available_models()
    print(f"可用模型: {models}")
    
    # 設定模型
    success = client.set_model("gemini-1.5-flash")
    print(f"設定模型結果: {success}")
    
    # 檢查目前模式
    print(f"目前模式: {client.get_current_mode()}")
    print(f"Live 模式是否啟動: {client.is_live_mode_active()}")
    
    # 生成回應（注意：這會實際調用 API，需要有效的 API key）
    # response = client.generate_response("Hello, how are you?")
    # print(f"AI 回應: {response}")
    
    print("✅ 傳統文字模式測試完成\n")

def demo_live_mode():
    """演示 Live 互動模式"""
    print("=== Live 互動模式演示 ===")
    
    client = GeminiClient()
    
    # 定義回調函數
    def on_text_received(text):
        print(f"[AI 文字] {text}")
    
    def on_audio_received(audio_data):
        print(f"[AI 音訊] 收到 {len(audio_data)} bytes 音訊資料")
    
    # 啟動 Live session（僅音訊模式，不使用攝影機）
    print("啟動 Live session...")
    success = client.start_live_session(
        video_mode="none",  # 不使用視訊
        voice_name="Zephyr",
        response_modalities=["AUDIO"],
        on_text_received=on_text_received,
        on_audio_received=on_audio_received
    )
    
    if success:
        print("✅ Live session 啟動成功")
        print(f"目前模式: {client.get_current_mode()}")
        print(f"Live 模式是否啟動: {client.is_live_mode_active()}")
        
        # 發送文字訊息到 Live session
        print("發送文字訊息...")
        client.send_text_to_live("Hello, this is a test message!")
        
        # 等待一段時間讓 session 運行
        print("讓 Live session 運行 5 秒...")
        time.sleep(5)
        
        # 停止 Live session
        print("停止 Live session...")
        stop_success = client.stop_live_session()
        print(f"停止結果: {stop_success}")
        print(f"Live 模式是否啟動: {client.is_live_mode_active()}")
    else:
        print("❌ Live session 啟動失敗")
    
    print("✅ Live 互動模式測試完成\n")

def demo_advanced_usage():
    """演示進階用法"""
    print("=== 進階用法演示 ===")
    
    client = GeminiClient()
    
    # 演示模式切換
    print("1. 初始狀態:")
    print(f"   模式: {client.get_current_mode()}")
    print(f"   Live 是否啟動: {client.is_live_mode_active()}")
    
    # 啟動 Live 模式
    print("\n2. 啟動 Live 模式:")
    success = client.start_live_session(video_mode="none")
    if success:
        print(f"   模式: {client.get_current_mode()}")
        print(f"   Live 是否啟動: {client.is_live_mode_active()}")
        
        # 停止 Live 模式
        print("\n3. 停止 Live 模式:")
        client.stop_live_session()
        print(f"   模式: {client.get_current_mode()}")
        print(f"   Live 是否啟動: {client.is_live_mode_active()}")
    
    print("✅ 進階用法演示完成\n")

def main():
    """主要演示函數"""
    print("GeminiClient 統一介面演示")
    print("=" * 50)
    
    try:
        # 演示傳統文字模式（向後相容性）
        demo_text_mode()
        
        # 演示 Live 互動模式
        # 注意：Live 模式需要有效的 API key 和相關依賴
        # demo_live_mode()
        
        # 演示進階用法
        demo_advanced_usage()
        
        print("🎉 所有演示完成！")
        
    except Exception as e:
        print(f"❌ 演示過程中發生錯誤: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()