#!/usr/bin/env python3
"""API quota checking tool for Gemini API.

This tool checks the validity of Gemini API keys and quota status,
providing diagnostic information and recommendations for quota management.
"""

import os
import sys
import time
from typing import Optional, Dict, Any, List

from google import genai
from google.genai import types

def check_api_key_validity(api_key: Optional[str] = None) -> Dict[str, Any]:
    """檢查 API 金鑰的有效性。
    
    Args:
        api_key: API 金鑰，如果為 None 則從環境變數讀取
        
    Returns:
        Dict[str, Any]: 包含檢查結果的字典，包括:
            - api_key_valid: 金鑰是否有效
            - api_key_source: 金鑰來源
            - error: 錯誤訊息（如果有）
            - models_available: 可用模型列表
    """
    result = {
        "api_key_valid": False,
        "api_key_source": None,
        "error": None,
        "models_available": []
    }
    
    # 獲取 API 金鑰
    if not api_key:
        api_key = os.getenv("GOOGLE_API_KEY") or os.getenv("GEMINI_API_KEY")
        if api_key:
            if os.getenv("GOOGLE_API_KEY"):
                result["api_key_source"] = "GOOGLE_API_KEY 環境變數"
            else:
                result["api_key_source"] = "GEMINI_API_KEY 環境變數"
        else:
            result["error"] = "未找到 API 金鑰。請設定 GOOGLE_API_KEY 或 GEMINI_API_KEY 環境變數。"
            return result
    else:
        result["api_key_source"] = "手動提供"
    
    # 測試 API 金鑰
    try:
        print(f"正在檢查 API 金鑰有效性...")
        print(f"API 金鑰來源: {result['api_key_source']}")
        print(f"API 金鑰後10位: ...{api_key[-10:] if api_key else 'None'}")
        
        # 建立客戶端
        client = genai.Client(
            api_key=api_key,
            http_options={"api_version": "v1beta"}
        )
        
        # 嘗試列出可用模型
        print("\n正在獲取可用模型列表...")
        models_response = client.models.list()
        
        if hasattr(models_response, 'models'):
            for model in models_response.models:
                if hasattr(model, 'name') and 'gemini' in model.name.lower():
                    model_name = model.name
                    if model_name.startswith('models/'):
                        model_name = model_name[7:]
                    result["models_available"].append(model_name)
        
        result["api_key_valid"] = True
        print(f"✓ API 金鑰有效！找到 {len(result['models_available'])} 個可用模型。")
        
    except Exception as e:
        error_msg = str(e)
        result["error"] = error_msg
        print(f"✗ API 金鑰檢查失敗: {error_msg}")
        
        # 分析錯誤類型
        if "invalid api key" in error_msg.lower() or "api key not valid" in error_msg.lower():
            print("\n錯誤類型: API 金鑰無效")
            print("解決方案: 請確認您的 API 金鑰是否正確。")
        elif is_quota_error(error_msg):
            print("\n錯誤類型: API 配額超限")
            print("解決方案: 請參考下方的配額管理建議。")
    
    return result

def test_basic_generation(api_key: Optional[str] = None) -> Dict[str, Any]:
    """測試基本的文字生成功能。
    
    Args:
        api_key: API 金鑰，如果為 None 則從環境變數讀取
        
    Returns:
        Dict[str, Any]: 包含測試結果的字典，包括:
            - test_passed: 測試是否通過
            - response: API 回應內容
            - error: 錯誤訊息（如果有）
            - model_used: 使用的模型名稱
    """
    result = {
        "test_passed": False,
        "response": None,
        "error": None,
        "model_used": "gemini-1.5-flash"
    }
    
    # 獲取 API 金鑰
    if not api_key:
        api_key = os.getenv("GOOGLE_API_KEY") or os.getenv("GEMINI_API_KEY")
        if not api_key:
            result["error"] = "未找到 API 金鑰"
            return result
    
    try:
        print("\n正在測試基本文字生成功能...")
        print(f"使用模型: {result['model_used']}")
        
        # 建立客戶端
        client = genai.Client(
            api_key=api_key,
            http_options={"api_version": "v1beta"}
        )
        
        # 測試簡單的文字生成
        test_prompt = "請回答：1+1等於多少？只需要回答數字。"
        print(f"測試提示詞: {test_prompt}")
        
        response = client.models.generate_content(
            model=result['model_used'],
            contents=test_prompt
        )
        
        # 提取回應
        if response and hasattr(response, 'text') and response.text:
            result["response"] = response.text.strip()
            result["test_passed"] = True
            print(f"✓ 文字生成測試成功！回應: {result['response']}")
        else:
            result["error"] = "API 回應格式異常"
            print("✗ 文字生成測試失敗: API 回應格式異常")
            
    except Exception as e:
        error_msg = str(e)
        result["error"] = error_msg
        print(f"✗ 文字生成測試失敗: {error_msg}")
        
        # 分析錯誤
        if is_quota_error(error_msg):
            print("\n配額超限錯誤詳情:")
            print(get_quota_error_details(error_msg))
    
    return result

def is_quota_error(error_msg: str) -> bool:
    """檢查是否為配額超限錯誤"""
    error_msg_lower = error_msg.lower()
    quota_indicators = [
        "quota",
        "rate limit",
        "too many requests",
        "resource exhausted",
        "1011",
        "429",
        "limit exceeded",
        "quota exceeded"
    ]
    
    return any(indicator in error_msg_lower for indicator in quota_indicators)

def get_quota_error_details(error_msg: str) -> str:
    """獲取配額錯誤的詳細資訊"""
    details = []
    
    if "1011" in error_msg:
        details.append("錯誤碼 1011: API 配額已用盡")
    elif "429" in error_msg:
        details.append("錯誤碼 429: 請求過於頻繁")
    elif "per minute" in error_msg.lower():
        details.append("超過每分鐘請求限制")
    elif "per day" in error_msg.lower():
        details.append("超過每日請求限制")
    
    return "\n".join(details) if details else "一般配額超限錯誤"

def print_quota_management_guide() -> None:
    """印出配額管理指南。
    
    顯示詳細的 API 配額管理建議和最佳實踐。
    """
    print("\n" + "="*60)
    print("📊 API 配額管理指南")
    print("="*60)
    
    print("\n1. 檢查配額使用情況:")
    print("   - 前往 Google AI Studio: https://aistudio.google.com/")
    print("   - 查看 'API Keys' 頁面的配額使用統計")
    
    print("\n2. 免費方案限制:")
    print("   - 每分鐘請求數限制 (RPM)")
    print("   - 每日請求數限制")
    print("   - 每分鐘 Token 數限制")
    
    print("\n3. 解決配額問題的方法:")
    print("   a) 短期解決方案:")
    print("      - 等待配額重置（通常為1分鐘或24小時）")
    print("      - 減少 API 呼叫頻率")
    print("      - 優化提示詞以減少 Token 使用")
    
    print("\n   b) 長期解決方案:")
    print("      - 升級到付費方案獲得更高配額")
    print("      - 實施請求快取機制")
    print("      - 使用批次處理減少呼叫次數")
    print("      - 實施請求佇列和速率限制")
    
    print("\n4. 最佳實踐:")
    print("   - 實施指數退避重試機制")
    print("   - 監控並記錄 API 使用情況")
    print("   - 在程式中加入配額檢查邏輯")
    print("   - 使用更高效的模型（如 gemini-1.5-flash）")
    
    print("\n5. 相關資源:")
    print("   - Google AI Studio: https://aistudio.google.com/")
    print("   - Gemini API 文件: https://ai.google.dev/docs")
    print("   - 定價資訊: https://ai.google.dev/pricing")
    print("="*60)

def main() -> None:
    """主程式進入點。
    
    執行完整的 API 配額檢查流程並提供診斷建議。
    """
    print("🔍 Gemini API 配額檢查工具")
    print("="*60)
    
    # 1. 檢查 API 金鑰
    key_check = check_api_key_validity()
    
    if key_check["api_key_valid"]:
        print(f"\n可用模型: {', '.join(key_check['models_available'][:5])}")
        if len(key_check['models_available']) > 5:
            print(f"... 還有 {len(key_check['models_available']) - 5} 個模型")
        
        # 2. 測試基本生成功能
        time.sleep(1)  # 避免太快發送請求
        generation_test = test_basic_generation()
        
        if not generation_test["test_passed"] and generation_test["error"]:
            if is_quota_error(generation_test["error"]):
                print("\n⚠️  檢測到配額超限問題！")
    else:
        print(f"\n錯誤: {key_check['error']}")
    
    # 3. 顯示配額管理指南
    print_quota_management_guide()
    
    # 4. 提供診斷建議
    print("\n💡 診斷建議:")
    if not key_check["api_key_valid"]:
        print("1. 請確認您的 API 金鑰是否正確")
        print("2. 確認環境變數設定是否正確")
        print("   Linux/Mac: export GOOGLE_API_KEY='your-api-key'")
        print("   Windows: set GOOGLE_API_KEY=your-api-key")
    elif key_check["api_key_valid"] and generation_test.get("error"):
        if is_quota_error(generation_test["error"]):
            print("1. 您的 API 金鑰有效，但配額已用盡")
            print("2. 請等待配額重置或考慮升級方案")
            print("3. 實施請求速率限制以避免此問題")
        else:
            print("1. API 金鑰有效，但發生其他錯誤")
            print("2. 請檢查網路連線和防火牆設定")
            print("3. 確認選擇的模型是否支援您的請求")

if __name__ == "__main__":
    main()