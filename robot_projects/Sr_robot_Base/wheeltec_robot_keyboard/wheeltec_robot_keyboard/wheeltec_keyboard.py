#!/usr/bin/env python
# coding=utf-8
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Darby Lim
# Author: 潘林陞 Ray (2025/09/23 保全機器人專案修改)

# ====================================================
# ========== Step 1 引入必要的模組 =============
# ====================================================

import os           # For 環境變數
import select       # For 非阻塞式輸入
import sys          # For sys.stdin
import rclpy        # ROS2 Python API

# 備註:
# select 模組提供一種高效的方式來監控多個 I/O 流（如檔案描述符、網路連接等）的狀態變化。
# 它允許程式在等待 I/O 操作完成時不會阻塞，從而提高效率。
# 在這段程式碼中，select 用於檢查標準輸入（sys.stdin）是否有可讀取的資料，監控鍵盤輸入，以實現非阻塞式的鍵盤輸入處理。
# 這樣，程式可以在等待使用者輸入的同時繼續執行其他任務，而不會被阻塞住。

from geometry_msgs.msg import Twist     # For 發佈速度話題
from rclpy.qos import QoSProfile        # For 設定 QoS 參數

# 備註: 
# geometry_msgs 是 ROS 中的一個標準訊息套件，包含了多種用於描述空間中幾何資訊的訊息類型。
# Twist 是 geometry_msgs 套件中的一個訊息類型，主要用於描述機器人的線速度和角速度。
# 它包含兩個主要的向量成員: linear 和 angular。
# - linear: 描述機器人在 x、y、z 三個方向上的線速度，通常以公尺每秒 (m/s) 為單位。
# - angular: 描述機器人在 x、y、z 三個方向上的角速度，通常以弧度每秒 (rad/s) 為單位。
# Twist 訊息通常用於控制機器人的運動，例如在移動基座中，透過發佈 Twist 訊息到特定的話題（如 /cmd_vel），來指示機器人如何移動和轉向。

# =====================================================
# ========== Step 2 初始化鍵值和速度定義 =============
# =====================================================

# 2.1 Windows 與 Linux 共用的鍵盤輸入設定
if os.name == 'nt':     # Windows 的 name 是 'nt'，Linux 和 macOS 的 name 是 'posix'
    import msvcrt       # For Windows 鍵盤輸入，msvcrt 是 Windows 的標準函式庫
else:
    import termios      # For Linux 鍵盤輸入，termios 是 POSIX 系統的終端控制模組
    import tty          # For Linux 鍵盤輸入，tty 是 POSIX 系統的終端控制模組

# 備註:
# 鍵盤輸入需要根據作業系統的不同來使用不同的模組。
# 在 Windows 系統中，使用 msvcrt 模組來處理鍵盤輸入。
# 在 Linux 和 macOS 系統中，使用 termios 和 tty 模組來處理鍵盤輸入。
# 這樣的條件式匯入確保了程式在不同作業系統上都能正確地處理鍵盤輸入。
# - nt: 代表 Windows 系統，意思是 "New Technology"，是 Windows NT 系列作業系統的縮寫
# - posix: 代表類 Unix 系統，如 Linux 和 macOS，遵循 POSIX 標準，是一組定義作業系統介面和行為的標準

# 2.2 定義機器人最大速度與速度增量
BURGER_MAX_LIN_VEL = 0.22   # 這是漢堡機器人的最大線速度，單位 m/s。不過我也不知道漢堡機器人是什麼東西，哈哈
BURGER_MAX_ANG_VEL = 2.84   # 這是漢堡機器人的最大角速度，單位 rad/s

WAFFLE_MAX_LIN_VEL = 0.26   # 這是華夫機器人的最大線速度，單位 m/s
WAFFLE_MAX_ANG_VEL = 1.82   # 這是華夫機器人的最大角速度，單位 rad/s

LIN_VEL_STEP_SIZE = 0.01    # 這是線速度的增量，單位 m/s
ANG_VEL_STEP_SIZE = 0.1     # 這是角速度的增量，單位 rad/s

# 2.3 提示訊息
msg = """
Control Your carrrrrrrrrr!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly
b : switch to OmniMode/CommonMode
CTRL-C to quit
"""
e = """
Communications Failed
"""

# 2.4 定義鍵值對應的移動/轉向方向
# 鍵值對應移動/轉向方向，(1,0)=前進，(0,1)=左轉，(0,-1)=右轉，(-1,0)=後退
moveBindings = {
        'i':( 1, 0),
        'o':( 1,-1),
        'j':( 0, 1),
        'l':( 0,-1),
        'u':( 1, 1),
        ',':(-1, 0),
        '.':(-1, 1),
        'm':(-1,-1),
           }

# 2.5 定義鍵值對應的速度增量
# 鍵值對應速度增量，(1.1,1.1)=線速度與角速度都增加10%，(0.9,0.9)=線速度與角速度都減少10%
speedBindings={
        'q':(1.1,1.1),
        'z':(0.9,0.9),
        'w':(1.1,1),
        'x':(0.9,1),
        'e':(1,  1.1),
        'c':(1,  0.9),
          }

# 2.6 定義取得鍵值的函數
speed = 0.2         # 默認移動速度 m/s
turn  = 1           # 默認轉向速度 rad/s


# ====================================================
# ========== Step 3 初始化鍵盤輸入 =============
# ====================================================

# 3.1 定義取得鍵值的函數
def get_key(settings):
    
    if os.name == 'nt':                                         # Windows
        return msvcrt.getch().decode('utf-8')                   # 取得鍵值並解碼成 utf-8 字串
    
    tty.setraw(sys.stdin.fileno())                              # Linux，tty.setraw() 讓終端進入原始模式
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)       # 監控標準輸入，等待 0.1 秒
    
    if rlist:                                                   # 如果有輸入
        key = sys.stdin.read(1)                                 # 讀取一個字元
    else:                                                       # 如果沒有輸入
        key = ''                                                # 設定 key 為空字串

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)   # 恢復終端設定
    return key                                                  # 回傳鍵值

# 3.2 定義列印目前速度的函數
def print_vels(speed, turn):
    print('currently:\tspeed {0}\t turn {1} '.format(           # 列印目前速度
        speed,
        turn))


# =========================================
# ========== Step 4 主程式 =============
# =========================================

# 定義主程式
def main():
    
    # 4.1 設定 ROS 參數
    settings = None                                             # 初始化 settings 變數
    if os.name != 'nt':                                         # 如果不是 Windows
        settings = termios.tcgetattr(sys.stdin)                 # 儲存目前終端設定
    rclpy.init()                                                # 初始化 rclpy

    qos = QoSProfile(depth=10)                                  # 設定 QoS 參數
    node = rclpy.create_node('wheeltec_keyboard')               # 建立 ROS2 節點
    pub = node.create_publisher(Twist, 'cmd_vel', qos)          # 建立發佈者，發佈 Twist 訊息到 cmd_vel 話題
    
    # 備註:
    # ROS2 節點是 ROS2 系統中的基本單位，負責執行特定的任務或功能。
    # 節點可以發佈和訂閱話題、提供和使用服務，並與其他節點進行通信。
    # 在這段程式碼中，建立了一個名為 'wheeltec_keyboard' 的節點，並且該節點包含一個發佈者，用於將 Twist 訊息發佈到 'cmd_vel' 話題，以控制機器人的運動。
    
    # 4.2 初始化變數
    speed = 0.2                     # 線速度初始值
    turn  = 1.0                     # 角速度初始值
    x      = 0.0                    # 前進/後退方向
    th     = 0.0                    # 左/右轉向方向
    count  = 0.0                    # 計數器
    target_speed = 0.0              # 目標線速度
    target_turn  = 0.0              # 目標角速度
    target_HorizonMove = 0.0        # 目標橫向移動速度
    control_speed = 0.0             # 實際控制的線速度
    control_turn  = 0.0             # 實際控制的角速度
    control_HorizonMove = 0.0       # 實際控制的橫向移動速度
    Omni = 0                        # 是否為全向移動模式，0=否，1=是
    
    # 4.3 主迴圈選單
    try:
        print(msg)                                         # 列印提示訊息
        print(print_vels(speed, turn))                     # 列印目前速度
        
        while(1):
            key = get_key(settings)                        # 取得鍵值
            
            # 按下 'b' 鍵，切換全向移動模式
            if key=='b':               
                Omni=~Omni
                if Omni: 
                    print("Switch to OmniMode")
                    moveBindings['.']=[-1,-1]              # 後退左轉
                    moveBindings['m']=[-1, 1]              # 前進右轉
                else:
                    print("Switch to CommonMode")
                    moveBindings['.']=[-1, 1]              # 前進左轉
                    moveBindings['m']=[-1,-1]              # 後退右轉

            # 判断键值是否在移動/轉向方向鍵值內
            if key in moveBindings.keys():                 # 如果鍵值在 moveBindings 字典中
                x  = moveBindings[key][0]                  # 前進/後退方向
                th = moveBindings[key][1]                  # 左/右轉向方向
                count = 0

            # 判断键值是否在速度增量鍵值內
            elif key in speedBindings.keys():              # 如果鍵值在 speedBindings 字典中
                speed = speed * speedBindings[key][0]      # 更新線速度
                turn  = turn  * speedBindings[key][1]      # 更新角速度
                count = 0                                  # 重置計數器
                print(print_vels(speed,turn))              # 列印目前速度

            # 按下空白鍵或 'k' 鍵，強制停止
            elif key == ' ' or key == 'k' :
                x  = 0
                th = 0.0
                control_speed = 0.0
                control_turn  = 0.0
                HorizonMove   = 0.0

            # 不明鍵值，相關變數置0
            else:
                count = count + 1                          # 如果按下的鍵不是上述定義的鍵，計數器加1
                if count > 4:                              # 如果計數器大於4，表示長期沒有有效輸入
                    x  = 0                                 # 前進/後退方向歸0
                    th = 0.0                               # 左/右轉向方向歸0
                if (key == '\x03'):                        # 按下 Ctrl-C 鍵，跳出迴圈
                    break

           # 根據速度與方向計算目標速度
            target_speed = speed * x                       # 目標線速度
            target_turn  = turn * th                       # 目標角速度
            target_HorizonMove = speed*th                  # 目標橫向移動速度

            # 平滑控制，計算線速度實際控制速度
            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.1 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.1 )
            else:
                control_speed = target_speed

            # 平滑控制，計算角速度實際控制速度
            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.5 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.5 )
            else:
                control_turn = target_turn

            # 平滑控制，計算橫向移動實際控制速度
            if target_HorizonMove > control_HorizonMove:
                control_HorizonMove = min( target_HorizonMove, control_HorizonMove + 0.1 )
            elif target_HorizonMove < control_HorizonMove:
                control_HorizonMove = max( target_HorizonMove, control_HorizonMove - 0.1 )
            else:
                control_HorizonMove = target_HorizonMove

            # 設定速度訊息
            twist = Twist() 
            if Omni==0:
                twist.linear.x  = control_speed; twist.linear.y = 0.0;  twist.linear.z = 0.0
                twist.angular.x = 0.0;             twist.angular.y = 0.0; twist.angular.z = control_turn
            else:
                twist.linear.x  = control_speed; twist.linear.y = control_HorizonMove; twist.linear.z = 0.0
                twist.angular.x = 0.0;             twist.angular.y = 0.0;                  twist.angular.z = 0.0
            
            # 發佈速度訊息
            pub.publish(twist)

    except Exception as e:
        print(e)

    # 4.4 程式結束前的清理工作
    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        pub.publish(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
