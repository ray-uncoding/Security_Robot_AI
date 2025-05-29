#!/usr/bin/env python3
"""
安全版本的主程式
僅啟動傳統功能，Live 模式需要手動啟動
"""

import os
import sys

# 設定環境變數以禁用 Live 模式自動測試
os.environ['DISABLE_LIVE_MODE'] = '1'

from ui import launch_ui

def main():
    print("啟動保全機器人控制台 (安全模式)")
    print("注意：Live 模式已禁用，如需使用請手動啟動")
    
    # 啟動 UI (其他系統模組將在 UI 中控制)
    launch_ui()

if __name__ == "__main__":
    main()