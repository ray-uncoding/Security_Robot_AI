#!/usr/bin/env python3
"""Find models that support Gemini Live API.

This tool searches for and tests various Gemini models to identify
which ones support the Live API functionality.
"""

import os
import asyncio
from typing import List, Optional

from google import genai
from google.genai import types

# API configuration
API_KEY = os.getenv("GOOGLE_API_KEY") or os.getenv("GEMINI_API_KEY")

def test_live_models() -> None:
    """測試不同的 Live 模型名稱。
    
    測試多個可能的 Live 模型名稱並驗證其連接能力。
    """
    
    print("=== 查找支持 Live API 的 Gemini 模型 ===\n")
    
    # 可能的 Live 模型名稱
    potential_live_models = [
        "models/gemini-2.0-flash-exp",
        "models/gemini-exp-1206", 
        "models/gemini-2.0-flash-experimental",
        "models/gemini-live",
        "models/gemini-1.5-flash-live",
        "models/gemini-pro-live",
        "gemini-2.0-flash-exp",
        "gemini-exp-1206",
        "gemini-2.0-flash-experimental",
        "gemini-live",
        "gemini-1.5-flash-live",
        "gemini-pro-live"
    ]
    
    client = genai.Client(
        api_key=API_KEY,
        http_options={"api_version": "v1beta"}
    )
    
    print("1. 嘗試列出所有模型（包括實驗性模型）")
    print("-" * 50)
    
    try:
        # 嘗試列出所有模型
        models_response = client.models.list()
        
        if hasattr(models_response, 'models'):
            all_models = []
            live_candidates = []
            
            for model in models_response.models:
                if hasattr(model, 'name'):
                    model_name = model.name
                    all_models.append(model_name)
                    
                    # 檢查可能的 Live 模型
                    if any(keyword in model_name.lower() for keyword in ['live', 'exp', 'experimental', 'flash-2', '2.0']):
                        live_candidates.append(model_name)
            
            print(f"找到 {len(all_models)} 個模型:")
            for model in all_models:
                print(f"  - {model}")
            
            print(f"\n可能的 Live 模型候選 ({len(live_candidates)} 個):")
            for model in live_candidates:
                print(f"  🎯 {model}")
                
        else:
            print("無法獲取模型列表")
            
    except Exception as e:
        print(f"列出模型時發生錯誤: {e}")
    
    print("\n" + "=" * 60)
    print("2. 測試常見的 Live 模型名稱")
    print("-" * 50)
    
    # 測試每個可能的模型
    for model_name in potential_live_models:
        print(f"\n測試模型: {model_name}")
        test_live_connection(client, model_name)

def test_live_connection(client: genai.Client, model_name: str) -> bool:
    """測試特定模型的 Live 連接。
    
    Args:
        client: Genai 客戶端實例
        model_name: 要測試的模型名稱
        
    Returns:
        bool: 連接成功返回 True，否則返回 False
    """
    try:
        # 創建最簡單的 Live 配置
        config = types.LiveConnectConfig(
            response_modalities=["AUDIO"]
        )
        
        # 嘗試同步連接測試（快速失敗）
        async def quick_test():
            try:
                async with client.aio.live.connect(model=model_name, config=config) as session:
                    print(f"  ✅ {model_name} - 連接成功！")
                    return True
            except Exception as e:
                error_msg = str(e)
                if "not found" in error_msg or "not supported" in error_msg:
                    print(f"  ❌ {model_name} - 不支持 Live API")
                elif "permission" in error_msg or "access" in error_msg:
                    print(f"  🔒 {model_name} - 權限不足")
                else:
                    print(f"  ⚠️  {model_name} - 其他錯誤: {error_msg[:100]}...")
                return False
        
        # 運行測試
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        result = loop.run_until_complete(quick_test())
        loop.close()
        
        return result
        
    except Exception as e:
        print(f"  💥 {model_name} - 測試失敗: {e}")
        return False

def get_model_details() -> None:
    """獲取模型詳細信息。
    
    顯示可能支援 Live API 的模型的詳細資訊。
    """
    print("\n" + "=" * 60)
    print("3. 獲取模型詳細信息")
    print("-" * 50)
    
    client = genai.Client(
        api_key=API_KEY,
        http_options={"api_version": "v1beta"}
    )
    
    try:
        models_response = client.models.list()
        
        if hasattr(models_response, 'models'):
            for model in models_response.models:
                if hasattr(model, 'name'):
                    model_name = model.name
                    
                    # 只顯示可能相關的模型詳細信息
                    if any(keyword in model_name.lower() for keyword in ['exp', '2.0', 'flash']):
                        print(f"\n模型: {model_name}")
                        
                        # 檢查模型屬性
                        if hasattr(model, 'supported_generation_methods'):
                            methods = model.supported_generation_methods
                            print(f"  支持的生成方法: {methods}")
                            
                            # 檢查是否支持 Live API 相關方法
                            if 'generateContent' in str(methods):
                                print("  ✅ 支持 generateContent")
                            if 'streamGenerateContent' in str(methods):
                                print("  ✅ 支持 streamGenerateContent") 
                            
                        if hasattr(model, 'display_name'):
                            print(f"  顯示名稱: {model.display_name}")
                            
                        if hasattr(model, 'description'):
                            print(f"  描述: {model.description[:200]}...")
    
    except Exception as e:
        print(f"獲取模型詳細信息時發生錯誤: {e}")

if __name__ == "__main__":
    test_live_models()
    get_model_details()
    
    print("\n" + "=" * 60)
    print("🔍 診斷建議:")
    print("1. 如果找到支持的模型，請更新 gemini_client.py 中的 LIVE_MODEL")
    print("2. 如果沒有找到支持的模型，可能需要：")
    print("   - 申請 Gemini Live API 的 Early Access")
    print("   - 使用不同的 API 金鑰")
    print("   - 等待 Live API 正式發布")
    print("3. 檢查 Google AI Studio 中的可用模型")