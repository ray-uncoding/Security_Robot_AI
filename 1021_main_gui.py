import sys
import os
import yaml
import json
import math
import subprocess
import time
import signal

from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QComboBox, QInputDialog, QFileDialog, QMessageBox, QSpinBox ,QScrollArea
from PyQt5.QtGui import QPixmap, QPainter, QColor, QMouseEvent, QPolygonF,QImage
from PyQt5.QtCore import Qt, pyqtSlot, QPointF, QRect, QTimer
from PIL import Image


class MapWindow(QMainWindow):
    # ====================================================
    # ========== Step 1 基本匯入與初始化 =============
    # ====================================================
    
    def __init__(self, yaml_files, pgm_files):
        
        super().__init__()                      # 初始化父類別 QMainWindow，這是 QT 視窗的基礎類別
        
        # 1.0 路徑初始化
        self.gui_ws_path = os.path.expanduser("/home/nvidia/workspace/Security_Robot_AI/gui_ws")
        os.makedirs(self.gui_ws_path, exist_ok=True)

        
        # 1.1 地圖視窗初始化
        self.file_loaded = False                # 紀錄是否載入過地圖檔案，初始為 False
        self.origin = [0, 0]                    # 初始化原點座標，預定為 (0,0)，可根據 YAML 檔案更新
        self.resolution = 0.05                  # 初始化地圖解析度（假設為 0.05 米/像素）
        self.recorded_points = []               # 用於保存記錄的點位
        self.directions = []                    # 用於保存點位方向
        self.save_points()                      # 嘗試保存初始空點位，save_points 方法內會處理空點位情況

        # 1.2 終端機狀態設定
        self.terminal_process = None                                # 用於保存終端機進程
        self.timer = QTimer(self)                                   # 初始化定時器
        self.timer.timeout.connect(self.check_terminal_status)      # 連接定時器超時信號到檢查函式
        self.timer.start(1000)                                      # 每 1 秒檢查一次終端機狀態
        
        self.processes = {
            "turn_on_wheeltec": None,
            "wheeltec_nav2": None,
            "waypoint": None,
            "ros2_keyboard_teleop": None,
            "insta_control": None,
            "ax8_control": None,
            "lidar": None,
            "nav2": None,
            "slam": None,
            "keyboard": None,
            "rviz2": None
        }

        # 1.3 鍵盤控制設定
        self.linear_speed = 0.2                                     # 初始線速度
        self.keyboard_mode = False                                  # 鍵盤模式初始為關閉

        # 1.4 地圖檔案設定
        # 此路徑在專案下的 robot_projects/Sr_robot_Base/wheeltec_robot_nav2/map
        # 而本檔案在根目錄下 1021_main_gui.py
        
        # 絕對路徑
        self.map_directory = "/home/nvidia/workspace/Security_Robot_AI/robot_projects/Sr_robot_Base/wheeltec_robot_nav2/map"
        self.yaml_files, self.pgm_files = self.scan_map_files(self.map_directory)   # 掃描該目錄下的 YAML 和 PGM 檔案，返回兩個列表
        
        # 1.5 視窗初始化
        self.start_wheeltec_nav2_process()      # 啟動 wheeltec_nav2 指令
        self.load_saved_points()                # 嘗試載入先前保存的點位，load_saved_points 方法內會處理檔案不存在情況
        
        # 1.6 設定視窗標題
        self.setWindowTitle('Sr Robot GUI 控制介面 V2.10.21')
        # self.setFixedSize(1200, 700)          # 設定固定視窗大小,避免使用者調整大小導致佈局錯亂
        # 嘗試自由大小調整
        self.setMinimumSize(1000, 700)          # 設定最小視窗大小
        self.setMaximumSize(1600, 1000)          # 設定最大視窗大小
        
        
        # ---------------------------------------------------------------------------------------------
        # ------------------------------------ 1.7 設定主視窗佈局 ------------------------------------
        # ---------------------------------------------------------------------------------------------
        
        # -----------------------------------------------------------------------------------
        # -              
        # -            [下拉式地圖選單]
        # -     ____________________________
        # -     |                           |
        # -     | [按鈕區域]      [地圖區域] |
        # -     |___________________________|
        # -
        # -             [資訊顯示區域]
        # -
        # ------------------------------------------------------------------------------------
        
        
        # 1.7.0 主佈局設定
        main_layout = QVBoxLayout()                                                                         # 主垂直佈局，QVBoxLayout 用於垂直排列元件

        # 1.7.1 建立下拉式選單，顯示地圖名稱
        self.combo_box = QComboBox(self)                                                                    # 初始化下拉選單元件，QComboBox 用於顯示可選項目列表
        self.combo_box.addItem("Select a map")                                                              # 新增空白選項
        self.combo_box.addItems([os.path.splitext(os.path.basename(yaml))[0] for yaml in self.yaml_files])  # 新增掃描到的 YAML 檔案名稱（不含副檔名）
        self.combo_box.currentIndexChanged.connect(self.update_map)                                         # 連接選購變更事件
        main_layout.addWidget(self.combo_box)

        # -----------  1.7.2 按鈕區域  ---------------
        # ------------------------------------------
        # -                                        -
        # -   START    -   KEYBOARD   -            -
        # -   STOP     -    HOME      -            -
        # -  SETPOINT  -    ZOOMIN    -            -
        # -   CLEAR    -   ZOOMOUT    -            -
        # -  SAVEFILE  -  DELETEFILE  -            -
        # -  LOADFILE  -  CANCELFILE  -            -
        # -   INSTA    -     AX8      -            -
        # -   INIT     -              -            -
        # -     [ 停留時間資訊欄 ]     -            -
        # ------------------------------------------

        # 1.7.2.0 建立按鈕並設定其樣式
        self.start_button           = QPushButton("START\n啟用")
        self.stop_button            = QPushButton("STOP\n停止")
        self.home_button            = QPushButton("HOME\n回原點")
        self.set_point_button       = QPushButton("SET POINT\n設定點位")
        self.clear_button           = QPushButton("CLEAR\n清除點位")
        self.zoom_in_button         = QPushButton("Zoom In\n放大")
        self.zoom_out_button        = QPushButton("Zoom Out\n縮小")
        self.save_file_button       = QPushButton("SAVE FILE\n儲存檔案")
        self.delete_file_button     = QPushButton("DELETE FILE\n刪除檔案")
        self.load_file_button       = QPushButton("LOAD FILE\n載入檔案")
        self.cancel_file_button     = QPushButton("CANCEL FILE\n取消檔案")
        self.keyboard_mode_button   = QPushButton("KEYBOARD\n鍵盤控制")
        self.insta_mode_button      = QPushButton("INSTA\n全景相機控制")
        self.ax8_mode_button        = QPushButton("AX8\n熱顯像儀控制")
        self.init_system_button     = QPushButton("INIT\n系統初始化")

        # 1.7.2.1 綁定按鈕事件
        self.start_button.clicked.connect(self.start_process)
        self.stop_button.clicked.connect(self.stop_process)
        self.home_button.clicked.connect(self.home_process)
        self.set_point_button.clicked.connect(self.toggle_set_point_mode)
        self.clear_button.clicked.connect(self.clear_points)
        self.zoom_in_button.clicked.connect(self.zoom_in)
        self.zoom_out_button.clicked.connect(self.zoom_out)
        self.save_file_button.clicked.connect(self.save_file_with_name)
        self.delete_file_button.clicked.connect(self.delete_file)
        self.load_file_button.clicked.connect(self.load_file)
        self.cancel_file_button.clicked.connect(self.cancel_file)
        self.keyboard_mode_button.clicked.connect(self.toggle_keyboard_mode)
        self.insta_mode_button.clicked.connect(self.toggle_insta_mode)
        self.ax8_mode_button.clicked.connect(self.toggle_ax8_mode)
        self.init_system_button.clicked.connect(self.initialize_system)

        # 1.7.2.2 設定按鈕大小和顏色
        self.start_button.setFixedSize(110, 60)
        self.start_button.setStyleSheet("background-color: green; color: white; font-size: 18px;")
        self.stop_button.setFixedSize(110, 60)
        self.stop_button.setStyleSheet("background-color: red; color: white; font-size: 18px;")
        self.home_button.setFixedSize(110, 60)
        self.home_button.setStyleSheet("background-color: blue; color: white; font-size: 18px;")
        self.set_point_button.setFixedSize(110, 60)
        self.set_point_button.setStyleSheet("background-color: gray; color: white; font-size: 18px;")
        self.clear_button.setFixedSize(110, 60)
        self.clear_button.setStyleSheet("background-color: orange; color: white; font-size: 18px;")
        self.zoom_in_button.setFixedSize(110, 60)
        self.zoom_in_button.setStyleSheet("background-color: orange; color: white; font-size: 18px;")
        self.zoom_out_button.setFixedSize(110, 60)
        self.zoom_out_button.setStyleSheet("background-color: orange; color: white; font-size: 18px;")
        self.save_file_button.setFixedSize(110, 60)
        self.save_file_button.setStyleSheet("background-color: teal; color: white; font-size: 18px;")
        self.delete_file_button.setFixedSize(110, 60)
        self.delete_file_button.setStyleSheet("background-color: darkorange; color: white; font-size: 18px;")
        self.load_file_button.setFixedSize(110, 60)
        self.load_file_button.setStyleSheet("background-color: purple; color: white; font-size: 18px;")
        self.cancel_file_button.setFixedSize(110, 60)
        self.cancel_file_button.setStyleSheet("background-color: darkred; color: white; font-size: 16px;")
        self.keyboard_mode_button.setFixedSize(110, 60)
        self.keyboard_mode_button.setStyleSheet("background-color: gray; color: white; font-size: 16px;")
        self.insta_mode_button.setFixedSize(110, 60)
        self.insta_mode_button.setStyleSheet("background-color: yellow; color: white; font-size: 16px;")
        self.ax8_mode_button.setFixedSize(110, 60)
        self.ax8_mode_button.setStyleSheet("background-color: red; color: white; font-size: 16px;")
        self.init_system_button.setFixedSize(110, 60)
        self.init_system_button.setStyleSheet("background-color: purple; color: white; font-size: 16px;")
        
        # 1.7.2.3 新增水平佈局，用於放置啟動和鍵盤控制按鈕，放置先後順序為 start_button 在前
        file_buttons_layout0 = QHBoxLayout()
        file_buttons_layout0.addWidget(self.start_button)
        file_buttons_layout0.addWidget(self.keyboard_mode_button)

        # 1.7.2.4 新增水平佈局，用於放置停止和回原點按鈕，放置先後順序為 stop_button 在前
        file_buttons_layout1 = QHBoxLayout()
        file_buttons_layout1.addWidget(self.stop_button)
        file_buttons_layout1.addWidget(self.home_button)

        # 1.7.2.5 新增水平佈局，用於放置儲存檔案和刪除檔案按鈕，放置先後順序為 save_file_button 在前
        file_buttons_layout2 = QHBoxLayout()
        file_buttons_layout2.addWidget(self.set_point_button)
        file_buttons_layout2.addWidget(self.zoom_in_button)

        # 1.7.2.6 新增水平佈局，用於放置載入檔案和取消檔案按鈕，放置先後順序為 load_file_button 在前
        file_buttons_layout3 = QHBoxLayout()
        file_buttons_layout3.addWidget(self.clear_button)
        file_buttons_layout3.addWidget(self.zoom_out_button)

        # 1.7.2.7 新增水平佈局，用於放置儲存檔案和刪除檔案按鈕，放置先後順序為 save_file_button 在前
        file_buttons_layout4 = QHBoxLayout()
        file_buttons_layout4.addWidget(self.save_file_button)
        file_buttons_layout4.addWidget(self.delete_file_button)

        # 1.7.2.8 新增水平佈局，用於放置載入檔案和取消檔案按鈕，放置先後順序為 load_file_button 在前
        file_buttons_layout5 = QHBoxLayout()
        file_buttons_layout5.addWidget(self.load_file_button)
        file_buttons_layout5.addWidget(self.cancel_file_button)

        # 1.7.2.9 新增水平佈局，用於放置 INSTA 和 AX8 按鈕，放置先後順序為 insta_mode_button 在前
        file_buttons_layout6 = QHBoxLayout()
        file_buttons_layout6.addWidget(self.insta_mode_button)
        file_buttons_layout6.addWidget(self.ax8_mode_button)
        
        file_buttons_layout7 = QHBoxLayout()
        file_buttons_layout7.addWidget(self.init_system_button)

        # 1.7.2.10 建立按鈕佈局
        button_layout = QVBoxLayout()                   # 垂直佈局，用於放置所有按鈕
        button_layout.setContentsMargins(5, 5, 5, 5)    # 設定邊距，邊距為 5 像素
        button_layout.setSpacing(20)                    # 設定按鈕間距，間距為 20 像素
        button_layout.addLayout(file_buttons_layout0)   # 添加 [啟動] 和 [鍵盤控制] 按鈕佈局
        button_layout.addLayout(file_buttons_layout1)   # 添加 [停止] 和 [回原點] 按鈕佈局
        button_layout.addLayout(file_buttons_layout2)   # 添加 [設定點位] 和 [放大] 按鈕佈局
        button_layout.addLayout(file_buttons_layout3)   # 添加 [清除] 和 [縮小] 按鈕佈局
        button_layout.addLayout(file_buttons_layout4)   # 添加 [儲存] 和 [刪除] 按鈕佈局
        button_layout.addLayout(file_buttons_layout5)   # 添加 [載入] 和 [取消] 按鈕佈局
        button_layout.addLayout(file_buttons_layout6)   # 添加 [INSTA] 和 [AX8] 按鈕佈局
        button_layout.addLayout(file_buttons_layout7)   # 添加 [系統初始化] 按鈕佈局

        # ---------------------------- 1.7.3 資訊顯示區域  ---------------------------
        # ---------------------------------------------------------------------------
        # -   世界座標: (-, -)   -        檔案名稱: 無               -   比例尺: 無   -
        # ---------------------------------------------------------------------------

        # 1.7.3.1 建立數值顯示區域
        self.value_label = QLabel("世界座標: (-, -)", self)         # 建立顯示世界座標的標籤
        self.value_label.setFixedSize(240, 30)                     # 設定標籤大小
        self.value_label.setStyleSheet("background-color: white; border: 1px solid black;")

        # 1.7.3.2 建立檔案名稱顯示區域
        self.file_name_label = QLabel("檔案名稱: 無", self)         # 建立顯示檔案名稱的標籤
        self.file_name_label.setFixedSize(700, 30)                 # 設定標籤大小
        self.file_name_label.setStyleSheet("background-color: white; border: 1px solid black;")

        # 1.7.3.3 建立比例尺顯示區域
        self.scale_label = QLabel("比例尺: 無", self)               # 建立顯示比例尺的標籤
        self.scale_label.setFixedSize(200, 30)                     # 設定標籤大小
        self.scale_label.setStyleSheet("background-color: white; border: 1px solid black;")

        # 1.7.3.4 新增水平佈局，放置數值、檔案名稱和比例尺
        info_layout = QHBoxLayout()
        info_layout.addWidget(self.value_label)                    # 添加世界座標標籤
        info_layout.addWidget(self.file_name_label)                # 添加檔案名稱標籤
        info_layout.addWidget(self.scale_label)                    # 添加比例尺標籤

        # 1.7.4 建立地圖顯示區域與滾動功能
        self.scroll_area = QScrollArea(self)                                    # 創建滾動區域
        self.label = MapLabel(self)                                             # 初始化地圖顯示區域
        self.scroll_area.setWidget(self.label)                                  # 設置 MapLabel 為滾動區域的子元件
        self.scroll_area.setWidgetResizable(True)                               # 允許滾動區域隨視窗大小改變
        self.scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOn)     # 永遠顯示水平滾動條
        self.scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)       # 永遠顯示垂直滾動條
        map_layout = QVBoxLayout()                                              # 垂直佈局，用於放置地圖顯示區域
        map_layout.addWidget(self.scroll_area)                                  # 添加滾動區域
        map_layout.setContentsMargins(0, 0, 0, 0)                               # 設定邊距為 0，讓地圖區域最大化利用空間

        # ----------------- 1.7.5 將按鈕佈局和地圖佈局新增至主佈局 -----------------------
        # -----------------------------------------------------------------------------------
        # -              
        # -            [下拉式地圖選單]
        # -     ____________________________
        # -     |                           |
        # -     | [按鈕區域]      [地圖區域] |
        # -     |___________________________|
        # -
        # -             [資訊顯示區域]
        # -
        # ------------------------------------------------------------------------------------
        
        # 1.7.5.1 組合按鈕區域和地圖區域到主佈局
        content_layout = QHBoxLayout()                           # 水平佈局，用於放置按鈕區域和地圖區域
        content_layout.addLayout(button_layout)                  # 添加按鈕區域佈局
        content_layout.addLayout(map_layout)                     # 添加地圖區域佈局
        main_layout.addLayout(content_layout)                    # 將內容佈局添加到主佈局中
        main_layout.addLayout(info_layout)

        # 1.7.6 設定中心視窗
        container = QWidget()                                    # 建立一個 QWidget 作為主視窗的容器
        container.setLayout(main_layout)                         # 設定主佈局到容器
        self.setCentralWidget(container)                         # 設定容器為主視窗的中心元件

        # 1.7.7 記錄模式狀態
        self.recording_mode = False                              # 點位記錄模式初始為關閉
        self.current_waypoint = None                             # 當前導航點位初始化為 None

        # 1.7.8 建立停留時間設定區域 (小時、分鐘、秒)
        self.stay_duration_label = QLabel('停留時間 (小時:分鐘:秒)', self)
        self.stay_duration_label.setFixedSize(150, 50)

        # 1.7.8.1 建立三個 QSpinBox 控制元件
        self.stay_duration_hours = QSpinBox(self)
        self.stay_duration_hours.setRange(0, 23)                 # 小時範圍 0 到 23
        self.stay_duration_hours.setValue(0)                     # 預設為 0 小時
        self.stay_duration_hours.setFixedSize(60, 50)

        self.stay_duration_minutes = QSpinBox(self)
        self.stay_duration_minutes.setRange(0, 59)               # 分鐘範圍 0 到 59
        self.stay_duration_minutes.setValue(0)                   # 預設為 0 分鐘
        self.stay_duration_minutes.setFixedSize(60, 50)

        self.stay_duration_seconds = QSpinBox(self)
        self.stay_duration_seconds.setRange(0, 59)               # 秒範圍 0 到 59
        self.stay_duration_seconds.setValue(10)                  # 預設為 10 秒
        self.stay_duration_seconds.setFixedSize(60, 50)

        # 1.7.8.2 建立一個橫向佈局來放置時間控制元件，並添加到按鈕佈局中
        time_layout = QHBoxLayout()
        time_layout.addWidget(self.stay_duration_hours)          # 添加小時控制元件
        time_layout.addWidget(self.stay_duration_minutes)        # 添加分鐘控制元件
        time_layout.addWidget(self.stay_duration_seconds)        # 添加秒數控制元件
        button_layout.addWidget(self.stay_duration_label)        # 添加停留時間標籤
        button_layout.addLayout(time_layout)                     # 添加時間控制元件佈局

    # ====================================================
    # ========== Step 2 各型事件 ==========================
    # ====================================================
    # 1. start_process: 啟動導航進程
    # 2. stop_process: 停止導航進程
    # 3. home_process: 回原點
    # 4. set_point_process: 切換設定點位模式
    # 5. clear_points: 清除所有點位
    # 6. zoom_in: 地圖放大
    # 7. zoom_out: 地圖縮小
    # 8. save_file_with_name: 以指定名稱儲存地圖檔案
    # 9. delete_file: 刪除指定地圖檔案
    # 10. load_file: 載入指定地圖檔案
    # 11. cancel_file: 取消目前載入的地圖檔案
    # 12. toggle_keyboard_mode: 切換鍵盤控制模式
    # 13. toggle_insta_mode: 切換 INSTA 全景相機控制模式
    # 14. toggle_ax8_mode: 切換 AX8 熱顯像儀控制模式
    # ====================================================
   
    # 2.1 啟動導航進程
    # 此函式將會啟動 ros2 的 wheeltec_robot_nav2 waypoint_testgui_time.py 腳本
    # 機器人底盤將會根據 ~/gui_ws/saved_points.json 檔案中的點位進行導航
    # 每個點位將會停留使用者在 GUI 中設定的時間
    
    @pyqtSlot()  # Decorator，表示這是一個槽函式，可以連接到信號
    def start_process(self):
        
        # 2.1.1 檢查是否退出設定點位模式
        if self.recording_mode:
            QMessageBox.warning(self, "警告", "請先退出設定點位模式（按一次 SET POINT 使其變為灰色）後再啟用！")
            return

        # 2.1.2 檢查是否選擇了地圖或載入了檔案
        if not self.combo_box.currentIndex() and not self.file_loaded:
            QMessageBox.warning(self, "警告", "請選擇地圖或載入檔案後再啟用！")
            return

        # 2.1.3 檢查 save_points.json 是否有數據
        save_points_path = os.path.join(self.gui_ws_path, "saved_points.json")

        # 2.1.3.1 檢查檔案是否存在
        if not os.path.exists(save_points_path):
            QMessageBox.warning(self, "警告", "未找到 saved_points.json 檔案，請確認點位是否已保存！")
            return

        # 2.1.3.2 讀取檔案並檢查內容
        with open(save_points_path, 'r') as file:
            data = json.load(file)
            if not data.get('points'):      # 檢查 'points' 欄位是否存在且有數據
                QMessageBox.warning(self, "警告", "saved_points.json 檔案中沒有有效的點位數據，請添加點位後再啟用！")
                return

        # 2.1.4 如果終端機進程不存在或已終止，啟動導航進程
        if self.terminal_process is None or self.terminal_process.poll() is not None:
            
            # 2.1.4.1 啟動前更新 JSON 檔案
            default_json_path = os.path.join(self.gui_ws_path, "saved_points.json")    # 預設點位檔案路徑
            self.save_points_to_file(default_json_path)                             # 呼叫 save_points_to_file，保存當前點位到預設檔案
            print(f"已更新導航點位檔案至: {default_json_path}")
            
            # 2.1.4.2 啟動導航進程
            # 指令如下:
            # 1. xterm
            # 2. -e
            # 3. bash -c 'cd /home/nvidia/workspace/Security_Robot_AI/robot_projects/Sr_robot_Base && source install/setup.bash &&
            #    ros2 launch wheeltec_robot_nav2 waypoint_testgui_time.py; exec bash'

            process_name = "waypoint"
            command = [
                "xterm", "-T", "waypoint", "-e",
                "bash", "-c",
                "source /home/nvidia/workspace/Security_Robot_AI/robot_projects/Sr_robot_Base/install/setup.bash && ros2 launch wheeltec_robot_nav2 waypoint_testgui_time.py; exec bash"
            ]
            self.toggle_process(process_name, command)

            print(f"啟動的終端機進程 PID: {self.processes[process_name].pid}")

        # 2.1.5 終端機進程已在運行，顯示提示訊息
        else:
            print("終端機進程已經在運行，無法再次啟動")
    
    
    # 2.2 停止導航進程
    @pyqtSlot()
    def stop_process(self):
        
        try:
            # 2.2.1 停止當前導航終端機進程
            if self.terminal_process:
                print(f"停止終端機進程 PID: {self.terminal_process.pid}")
                self.terminal_process.terminate()                               # 終止進程
                self.terminal_process.wait()                                    # 等待進程完全終止
                self.terminal_process = None                                    # 重置狀態
            else:
                print("沒有導航進程在運行")

            # 2.2.2 等待進程完全終止
            for _ in range(5):  # 最多等待 5 秒
                time.sleep(1)
                if self.terminal_process is None or self.terminal_process.poll() is not None:
                    break
            else:
                print("進程未完全終止，請手動檢查進程狀態。")

            print("導航進程已停止")

            # 2.2.3 停用 /bt_navigator，這個是 ROS2 的行為樹導航節點
            print("正在停用 /bt_navigator...")
            deactivate_command = "ros2 lifecycle set /bt_navigator deactivate"
            subprocess.run(
                deactivate_command, shell=True, executable='/bin/bash', check=True,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            print("/bt_navigator 已成功停用")

        except subprocess.CalledProcessError as e:
            print(f"執行停用 /bt_navigator 時出現錯誤: {e.stderr.decode('utf-8')}")
        except Exception as e:
            print(f"停止過程中出現未知錯誤: {e}")


    # 2.3 回原點
    @pyqtSlot()
    def home_process(self):
        
        # 2.3.1 清除之前的所有點位，並儲存原點作為唯一導航點
        print("回原點按鈕被點擊，清除所有點位並儲存原點")
        origin_waypoint = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0,
            "stay_duration": 100000  # 默認停留時間 10 秒
        }
        
        # 2.3.2 清除之前的點位
        self.recorded_points = [origin_waypoint]  # 只儲存原點為唯一點位
        
        # 2.3.3 儲存更新後的點位到 saved_points.json
        self.save_points()
        print("已清除所有點位並將原點儲存為新點位")


    # 2.4 切換設定點位模式
    @pyqtSlot()
    def toggle_set_point_mode(self):
        
        # 2.4.1 切換點位記錄模式狀態
        self.recording_mode = not self.recording_mode
        
        # 2.4.2 如果進入記錄模式，改變按鈕顏色並清空點位（若未載入檔案）
        if self.recording_mode:
            self.set_point_button.setStyleSheet("background-color: orange; color: white; font-size: 18px;")
            
            # 2.4.2.1 若已載入檔案，不清空點位
            if self.file_loaded:
                print("已進入點位記錄模式（保留載入的點位）")
            
            # 2.4.2.2 若未載入檔案，清空點位
            else:
                self.recorded_points = []
                print("已進入點位記錄模式（未載入檔案，清空點位）")
        
        # 2.4.3 如果退出記錄模式，改變按鈕顏色並儲存點位
        else:
            self.set_point_button.setStyleSheet("background-color: gray; color: white; font-size: 18px;")
            self.save_points()
            self.window().value_label.setText(f"世界座標:  (-, -)")
            print("已退出點位記錄模式")


    # 2.5 清除所有點位
    @pyqtSlot()
    def clear_points(self):
        
        # 2.5.1 清除記錄的點位和方向，並更新地圖顯示
        self.recorded_points = []                   # 清空記錄的點位列表
        self.directions = []                        # 清空方向箭頭列表
        self.label.clear()                          # 清空地圖顯示
        self.save_points()                          # 儲存空的點位列表到檔案
        print("所有記錄的點位已清除")


    # 2.6 地圖放大
    @pyqtSlot()
    def zoom_in(self):
       
        # 2.6.1 調整縮放比例並更新地圖顯示
        self.label.scale_factor *= 1.2  
        self.label.update_image(
            self.label.current_map_file,
            self.label.image_height,
            self.label.origin,
            self.label.resolution,
            self.label.scale_factor
        )
        print(f"縮放後的比例 (放大): {self.label.scale_factor}")
    
    
    # 2.7 地圖縮小
    @pyqtSlot()
    def zoom_out(self):

        # 2.7.1 調整縮放比例並更新地圖顯示
        self.label.scale_factor /= 1.2
        self.label.update_image(
            self.label.current_map_file,
            self.label.image_height,
            self.label.origin,
            self.label.resolution,
            self.label.scale_factor
        )
        print(f"縮放後的比例 (縮小): {self.label.scale_factor}")
    
    
    # 2.8 儲存檔案事件
    def save_file_with_name(self):
        
        # 2.8.1 透過直接讀取標籤，取得目前選擇的地圖名稱
        current_map_name = self.combo_box.currentText()
        
        # 2.8.2 檢查是否選擇了有效的地圖
        if current_map_name == "Select a map":
            QMessageBox.warning(self, "警告", "請先選擇一個地圖再進行儲存操作！")           # 備註，這個訊息框會顯示在主視窗之上
            self.label.points.clear()
            self.label.directions.clear()
            return

        # 2.8.3 根據是否已載入檔案，決定覆蓋或另存新檔案
        if self.file_loaded and self.label.current_file_name:                                       # 已載入檔案且有檔案名稱
            msg_box = QMessageBox(self)                                                             # 建立訊息框
            msg_box.setWindowTitle("儲存檔案")                                                       # 設定視窗標題
            msg_box.setText(f"已載入檔案：{self.label.current_file_name}\n您想要覆蓋此檔案嗎？")       # 設定訊息內容
            msg_box.setStandardButtons(QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel)       # 設定按鈕，包含 Yes、No 和 Cancel
            msg_box.setDefaultButton(QMessageBox.Yes)                                               # 預設按鈕為 Yes
            response = msg_box.exec_()

            # 2.8.3.1 YES，覆蓋原檔案
            if response == QMessageBox.Yes:
                save_path = os.path.join(self.gui_ws_path, self.label.current_file_name)                
                self.save_points_to_file(save_path)
                print(f"已覆蓋原檔案：{save_path}")

            # 2.8.3.2 NO，另存為新檔案
            elif response == QMessageBox.No:
                file_name, ok = QInputDialog.getText(self, "另存為", "請輸入新檔案名稱（不含副檔名）:")
                if ok and file_name:
                    # 2.8.3.2.1 儲存至新檔案 
                    save_path = os.path.join(self.gui_ws_path, f"{current_map_name}_{file_name}.json")
                    self.save_points_to_file(save_path)
                    print(f"已儲存至新檔案：{save_path}")
                    # 2.8.3.2.2 關閉點位記錄模式
                    self.recording_mode = False
                    self.set_point_button.setStyleSheet("background-color: gray; color: white; font-size: 18px;")
                    print("已自動退出點位記錄模式")

            # 2.8.3.3 CANCEL
            else:
                print("取消儲存操作")
        else:
            # 2.8.4 未載入檔案，直接要求輸入新檔案名稱
            file_name, ok = QInputDialog.getText(self, "儲存檔案", "請輸入檔案名稱（不含副檔名）:")
            if ok and file_name:
                # 2.8.4.1 儲存至新檔案
                save_path = os.path.join(self.gui_ws_path, f"{current_map_name}_{file_name}.json")
                self.save_points_to_file(save_path)
                print(f"點位已儲存至: {save_path}")
                # 2.8.4.2 關閉點位記錄模式
                self.recording_mode = False
                self.set_point_button.setStyleSheet("background-color: gray; color: white; font-size: 18px;")
                print("已自動退出點位記錄模式")
            else:
                print("取消儲存操作")


    # 2.9 刪除檔案事件
    def delete_file(self):
        
        # 2.9.1 開啟檔案選擇對話框，讓使用者選擇要刪除的檔案
        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getOpenFileName(
            self, "選擇要刪除的檔案", self.gui_ws_path, "JSON Files (*.json)", options=options
        )
        if not file_path:  # 如果未選擇檔案
            print("未選擇檔案")
            return

        # 2.9.2 顯示確認刪除對話框
        msg_box = QMessageBox(self)
        msg_box.setWindowTitle("確認刪除")
        msg_box.setText(f"是否確定要刪除檔案：{os.path.basename(file_path)}？")
        msg_box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        msg_box.setDefaultButton(QMessageBox.No)
        response = msg_box.exec_()

        # 2.9.3 根據使用者回應執行刪除操作
        if response == QMessageBox.Yes:
            try:
                os.remove(file_path)
                print(f"成功刪除檔案: {file_path}")
            except Exception as e:
                print(f"刪除檔案失敗: {e}")
        else:
            print("取消刪除操作")


    # 2.10 載入檔案事件
    def load_file(self):
        
        # 2.10.1 開啟檔案選擇對話框，讓使用者選擇要載入的檔案
        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getOpenFileName(
            self, "選擇檔案", self.gui_ws_path, "JSON Files (*.json)", options=options
        )
        if not file_path:
            print("未選擇檔案")
            return
        
        # 2.10.2 嘗試讀取並載入選擇的檔案
        try:

            # 2.10.2.1 讀取檔案內容
            with open(file_path, 'r') as file:
                data = json.load(file)

            # 2.10.2.2 驗證檔案結構
            if 'points' not in data or not isinstance(data['points'], list):
                raise ValueError("JSON 檔案格式不正確，缺少 'points' 欄位或結構不符")

            # 2.10.2.3 將目前載入的檔案內容同步到 saved_points.json
            default_json_path = os.path.join(self.gui_ws_path, "saved_points.json")        # 預設點位檔案路徑
            with open(default_json_path, 'w') as default_file:                          # 開啟預設檔案進行寫入
                json.dump(data, default_file, indent=4)                                 # 將載入的數據寫入預設檔案
            print(f"已同步檔案內容到: {default_json_path}")
            self.save_points()                                                          # 呼叫 save_points，確保 GUI 狀態同步

            # 2.10.2.4 從檔案名稱中提取地圖名稱
            file_name = os.path.basename(file_path)                                     # 取得檔案名稱
            map_name = file_name.split('_')[0]                                          # 假設檔案名稱格式為 mapname_xxx.json
            print(f"從檔案名稱提取的地圖名稱: {map_name}")
            self.label.set_file_name(os.path.basename(file_path))                       # 設定目前檔案名稱以顯示在 GUI 上

            # 2.10.2.5 在下拉選單中自動選擇對應地圖
            index = self.combo_box.findText(map_name)
            if index == -1:
                QMessageBox.warning(self, "警告", f"未找到與地圖名稱 '{map_name}' 匹配的地圖")
                self.label.clear_map()                                                  # 隱藏地圖
                self.label.clear()                                                      # 確保清空地圖
                return
            self.combo_box.setCurrentIndex(index)

            # 2.10.2.6 調用地圖更新邏輯
            self.update_map(index)

            # 2.10.2.7 清空新增點位，但保留載入的點位
            self.recorded_points = []                       # 清空記錄的點位列表
            self.label.new_points = []                      # 新增一個列表來記錄新增點
            self.label.points = []                          # 清空繪製的點（重新繪製載入點）

            # 2.10.2.8 設定地圖縮放比例為1.0
            self.label.scale_factor = 1.0                   # 重置縮放比例為 1.0

            # 2.10.2.9 處理載入的點位與方向
            self.label.loaded_points = []                   # 紀錄載入的點位
            self.label.directions.clear()                   # 清空方向箭頭後重繪

            # 2.10.2.10 處理載入的點位與方向
            for point in data['points']:
                # 驗證點位是否完整
                if not all(key in point for key in ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']):
                    print(f"點位數據不完整，跳過: {point}")
                    continue

                # 計算像素位置
                pixel_pos = self.label.world_to_pixel(QPointF(point['x'], point['y']))
                self.label.loaded_points.append(pixel_pos)

                # 計算方向箭頭的終點
                try:
                    angle = quaternion_to_angle(point['qx'], point['qy'], point['qz'], point['qw'])
                    arrow_length = 50  # 固定箭頭長度
                    end_point = pixel_pos + QPointF(
                        arrow_length * math.cos(angle),
                        arrow_length * math.sin(angle)
                    )
                    self.label.directions.append((pixel_pos, end_point))
                except Exception as e:
                    print(f"計算方向時出錯: {e}, 點位: {point}")
                    continue

            # 2.10.2.11 將載入的點保存到 `recorded_points`（作為基礎點位）
            self.recorded_points = data['points']
            self.file_loaded = True

            print(f"成功載入檔案: {file_path}")
            self.save_points()
            self.label.update()  # 觸發重新繪製
        except Exception as e:
            print(f"載入檔案失敗: {e}")
 
 
    # 2.11 取消載入檔案事件
    def cancel_file(self):
        
        # 2.11.1 檢查是否有檔案已載入
        if not self.file_loaded:
            print("尚未載入任何檔案，無需取消")
            self.label.clear_map()                      # 隱藏地圖
            self.combo_box.setCurrentIndex(0)           # 重置地圖選單
            self.label.current_map_file = None          # 清除當前地圖檔案
            return

        # 2.11.2 重置檔案載入狀態
        self.file_loaded = False                        # 重置檔案載入狀態
        self.recorded_points = []                       # 清空點位
        self.label.points.clear()                       # 清空地圖上的點位
        self.label.directions.clear()                   # 清空地圖上的方向箭頭
        self.label.set_file_name(None)                  # 清除檔案名稱
        self.combo_box.setCurrentIndex(0)               # 重置地圖選單
        self.label.clear_map()                          # 隱藏地圖
        print(f"已取消載入的檔案：{self.label.current_file_name}，所有點位及檔案名稱已清空")


    # 2.12 切換鍵盤控制模式
    @pyqtSlot()
    def toggle_keyboard_mode(self):
        try:
            print("啟動 ROS2 鍵盤控制進程...")
            
            # 2.12.1.1 啟動鍵盤進程
            # 指令如下:
            # 1. xterm
            # 2. -e
            # 3. bash -c 'cd /home/nvidia/workspace/Security_Robot_AI/robot_projects/Sr_robot_Base && source install/setup.bash &&
            #    ros2 run wheeltec_robot_keyboard wheeltec_keyboard.py; exec bash'

            self.keyboard_mode_button.setText("KEYBOARD MODE\n啟動中...")
            self.keyboard_mode_button.setStyleSheet("background-color: green; color: white; font-size: 16px;")
            
            process_name = "ros2_keyboard_teleop"
            command = [
                "xterm", "-T", "Keyboard Teleop", "-e",  # -T 設置視窗標題
                "bash", "-c", 
                "source /home/nvidia/workspace/Security_Robot_AI/robot_projects/Sr_robot_Base/install/setup.bash && ros2 run wheeltec_robot_keyboard wheeltec_keyboard; exec bash"
            ]
            self.toggle_process(process_name, command)

            print(f"啟動的終端機進程 PID: {self.processes[process_name].pid}")

            self.keyboard_mode_button.setText("KEYBOARD MODE\n關閉")
            self.keyboard_mode_button.setStyleSheet("background-color: gray; color: white; font-size: 16px;")
            print(f"鍵盤控制已啟動 (PID: {self.processes[process_name].pid})")
        except Exception as e:
            print(f"啟動鍵盤控制失敗: {e}")


    # 2.13 切換 INSTA 全景相機控制模式
    @pyqtSlot()
    def toggle_insta_mode(self):
        
        try:
            print("啟動 ROS2 INSTA 全景相機控制進程...")

            # 2.13.1.1 啟動全景相機進程
            # 指令如下:
            # 1. xterm
            # 2. -e
            # 3. bash -c 'python  Insta_OpenCV/trun_on_insta_OCR.py; exec bash'

            process_name = "insta_control"
            command = [
                "xterm", "-T", "INSTA Control", "-e",  # -T 設置視窗標題
                "bash", "-c", 
                "python3 /home/nvidia/workspace/Security_Robot_AI/Insta_OpenCV/trun_on_insta_OCR.py; exec bash"
            ]
            self.toggle_process(process_name, command)

            print(f"啟動的終端機進程 PID: {self.processes[process_name].pid}")

            self.insta_mode_button.setText("INSTA MODE\n關閉")
            self.insta_mode_button.setStyleSheet("background-color: green; color: white; font-size: 16px;")
            print(f"INSTA 控制已啟動 (PID: {self.processes[process_name].pid})")
        except Exception as e:
            print(f"啟動 INSTA 控制失敗: {e}")


    # 2.14 切換 AX8 熱顯像儀控制模式
    @pyqtSlot()
    def toggle_ax8_mode(self):
        try:
            print("啟動 ROS2 AX8 熱顯像儀控制進程...")

            # 2.14.1.1 啟動 AX8 熱顯像儀進程
            # 指令如下:
            # 1. xterm
            # 2. -e
            # 3. bash -c 'python  ax8/ax8_worker.py; exec bash'

            process_name = "ax8_control"
            command = [
                "xterm", "-T", "AX8 Control", "-e",  # -T 設置視窗標題
                "bash", "-c", 
                "python3 /home/nvidia/workspace/Security_Robot_AI/ax8/ax8_worker.py; exec bash"
            ]
            self.toggle_process(process_name, command)

            print(f"啟動的終端機進程 PID: {self.processes[process_name].pid}")

            self.ax8_mode_button.setText("AX8 MODE\n關閉")
            self.ax8_mode_button.setStyleSheet("background-color: green; color: white; font-size: 16px;")
            print(f"AX8 控制已啟動 (PID: {self.processes[process_name].pid})")
        except Exception as e:
            print(f"啟動 AX8 控制失敗: {e}")


    # 2.15 初始化並啟動 Wheeltec ROS2 進程
    @pyqtSlot()
    def start_wheeltec_nav2_process(self):

        print("啟動 wheeltec_nav2 進程")

        try:

            # 2.15.1.1 啟動 wheeltec_nav2 進程
            # 指令如下:
            # 1. xterm
            # 2. -e
            # 3. bash -c ''

            process_name = "wheeltec_nav2"
            command = [
                "xterm", "-T", "wheeltec_nav2", "-e",  # -T 設置視窗標題
                "bash", "-c", 
                "source /home/nvidia/workspace/Security_Robot_AI/robot_projects/Sr_robot_Base/install/setup.bash && ros2 launch wheeltec_robot_nav2 wheeltec_nav2.launch.py; exec bash"
            ]
            
            self.toggle_process(process_name, command)
            print(f"wheeltec_nav2 控制已啟動 (PID: {self.processes[process_name].pid})")
        except Exception as e:
            print(f"啟動 wheeltec_nav2 控制失敗: {e}")

    # 2.16 系統初始化流程
    @pyqtSlot()
    def initialize_system(self):
        """
        系統初始化流程:
        1. 啟動雷達
        2. 啟動導航系統
        3. 開始建圖
        4. 儲存地圖
        """
        try:
            # 按鈕變色提示正在執行
            self.init_system_button.setStyleSheet("background-color: green; color: white; font-size: 16px;")
            self.init_system_button.setText("INIT\n初始化中...")

            # 1. 啟動雷達
            #print("正在啟動雷達...")
            #lidar_command = [
            #    "xterm", "-e",
            #    "bash -c 'source /home/nvidia/workspace/Security_Robot_AI/robot_projects/Sr_robot_Base/install/setup.bash && ros2 launch turn_on_wheeltec_robot robotandlidar.launch.py; exec bash'"
            #]
            #self.toggle_process("lidar", lidar_command)

            # 等待雷達啟動
            #time.sleep(3)

            # 2. 啟動導航系統
            # 他會一併啟動 trun_on_wheeltec_robot 和 lidar 節點
            print("正在啟動導航系統...")
            nav_command = [
                "xterm", "-e",
                "bash -c 'source /home/nvidia/workspace/Security_Robot_AI/robot_projects/Sr_robot_Base/install/setup.bash && ros2 launch wheeltec_robot_nav2 wheeltec_nav2.launch.py; exec bash'"
            ]
            self.toggle_process("nav2", nav_command)

            # 等待導航系統啟動
            time.sleep(3)

            # 3. 開始建圖
            print("正在啟動建圖程序...")
            slam_command = [
                "xterm", "-e",
                "bash -c 'source /home/nvidia/workspace/Security_Robot_AI/robot_projects/Sr_robot_Base/install/setup.bash && ros2 launch slam_toolbox online_sync_launch.py; exec bash'"
            ]
            self.toggle_process("slam", slam_command)

            # 4. 打開鍵盤控制，方便使用者操作機器人進行建圖
            print("啟動鍵盤控制...")
            keyboard_command = [
                "xterm", "-e",
                "bash -c 'source /home/nvidia/workspace/Security_Robot_AI/robot_projects/Sr_robot_Base/install/setup.bash && ros2 run wheeltec_robot_keyboard wheeltec_keyboard; exec bash'"
            ]
            self.toggle_process("keyboard", keyboard_command)
            
            # 5. 打開 rviz2 以便觀察建圖過程
            # 啟用設定檔案 /home/nvidia/workspace/Security_Robot_AI/robot_projects/Sr_robot_Base/wheeltec_robot_nav2/rviz/wheeltec.rviz
            print("啟動 rviz2 以觀察建圖過程...")
            rviz_command = [
                "xterm", "-e",
                "bash -c 'source /home/nvidia/workspace/Security_Robot_AI/robot_projects/Sr_robot_Base/install/setup.bash && ros2 run rviz2 /home/nvidia/workspace/Security_Robot_AI/robot_projects/Sr_robot_Base/wheeltec_robot_nav2/rviz/wheeltec.rviz; exec bash'"
            ]
            self.toggle_process("rviz2", rviz_command)
            

            # 提示使用者
            QMessageBox.information(self, "系統初始化", 
                "系統已完成初始化！\n\n" +
                "1. 請使用鍵盤控制機器人繞行環境以完成建圖\n" +
                "2. 建圖完成後，請在終端中執行以下命令儲存地圖：\n" +
                "   ros2 run nav2_map_server map_saver_cli -f <地圖名稱>\n" +
                "3. 儲存完成後，請重啟程式以載入新地圖"
            )

            # 恢復按鈕狀態
            self.init_system_button.setStyleSheet("background-color: purple; color: white; font-size: 16px;")
            self.init_system_button.setText("INIT\n系統初始化")

        except Exception as e:
            print(f"初始化過程出錯: {e}")
            QMessageBox.critical(self, "錯誤", f"初始化過程發生錯誤：\n{str(e)}")
            self.init_system_button.setStyleSheet("background-color: purple; color: white; font-size: 16px;")
            self.init_system_button.setText("INIT\n系統初始化")


    # 通用函式: 啟動或停止子進程
    def toggle_process(self, process_name, command):        
        
        # 通用函式，用於啟動或停止一個子進程。

        # process_name: 為這個進程指定的唯一名稱 (e.g., 'ros2_navigation')
        # command: 一個包含命令和參數的列表 (list for subprocess.Popen)

        process = self.processes.get(process_name)  # 如果鍵不存在會返回 None

        # 檢查進程是否已在執行
        
        # 如果 process_name 在字典中，且對應的進程仍在運行
        # .poll() 用於檢查進程是否結束，返回 None 表示進程仍在運行
        if process is not None and process.poll() is None: 

            print(f"正在停止進程 '{process_name}'...")
            
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)              # 終止進程組
                process.wait(timeout=3)                                         # 等待終止
            except ProcessLookupError:
                print(f"進程 '{process_name}' 已經不存在。")
            except subprocess.TimeoutExpired:
                print(f"終止超時，強制終止進程組 '{process_name}'...")
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)              # 強制殺掉進程組
            except Exception as e:
                print(f"關閉進程時發生錯誤: {e}")
                process.kill()

            print(f"進程 '{process_name}' 已停止。")
            self.processes[process_name] = None                                 # 停止後，把它重置回「佔位符 None」
        
        # 如果進程未在執行，則啟動它    
        else:
            print(f"正在啟動進程 '{process_name}'...")
            try:
                process = subprocess.Popen(command, preexec_fn=os.setsid)       # 啟動新進程並創建新的進程組
                self.processes[process_name] = process                          # 將進程物件存入字典
                print(f"進程 '{process_name}' 已啟動 (PID: {process.pid})。")
            except FileNotFoundError:
                print(f"錯誤: 命令 '{command[0]}' 找不到。請確認已安裝並在 PATH 中。")
            except Exception as e:
                print(f"啟動進程 '{process_name}' 時發生錯誤: {e}")

    # 關閉視窗事件
    def closeEvent(self, event):

        print("視窗關閉事件觸發，正在清理所有背景進程...")
        
        # 取得所有正在運行的進程名稱
        # 使用 list() 是為了避免在迭代過程中修改字典
        running_process_names = list(self.processes.keys())
        
        for name in running_process_names:
            # 從字典中取得進程物件
            process = self.processes.get(name)
            
            # 檢查進程物件是否存在，且仍在運行
            if process and process.poll() is None:
                print(f"正在停止進程 '{name}' (PID: {process.pid})...")
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    process.wait(timeout=2)
                    print(f"進程 '{name}' 已成功終止。")
                except AttributeError:
                    print(f"警告: 系統不支援 killpg，使用 process.kill() 強制終止 '{name}'。")
                    process.kill()
                except subprocess.TimeoutExpired:
                    print(f"終止 '{name}' 超時，強制終止...")
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except Exception as e:
                    print(f"關閉進程 '{name}' 時發生錯誤: {e}，嘗試強制終止...")
                    process.kill()

        print("所有進程已清理完畢。")
        event.accept()
    
    # 儲存點位到 saved_points.json
    def save_points(self):
        """儲存記錄的點位到檔案，並限制數值小數點後 15 位，包含停留時間"""
        print('儲存的點位為:', self.recorded_points)

        # 格式化點位資料
        rounded_points = []
        for point in self.recorded_points:
            rounded_point = {
                "x": round(point["x"], 15),
                "y": round(point["y"], 15),
                "z": round(point["z"], 15),
                "qx": round(point["qx"], 15),
                "qy": round(point["qy"], 15),
                "qz": round(point["qz"], 15),
                "qw": round(point["qw"], 15),
                "stay_duration": int(point["stay_duration"])  # 儲存停留時間
            }
            rounded_points.append(rounded_point)

        # 將格式化後的點位資料存入 saved_points.json
        with open(os.path.join(self.gui_ws_path, 'saved_points.json'), 'w') as file:
            json.dump({'points': rounded_points}, file, indent=4)

        print("已將點位儲存到 saved_points.json")
    
    
    # 儲存點位到指定檔案路徑
    def save_points_to_file(self, file_path):
        """儲存記錄的點位到指定檔案路徑"""
        rounded_points = [
            {
                "x": round(point["x"], 15),
                "y": round(point["y"], 15),
                "z": round(point["z"], 15),
                "qx": round(point["qx"], 15),
                "qy": round(point["qy"], 15),
                "qz": round(point["qz"], 15),
                "qw": round(point["qw"], 15),
                "stay_duration": int(point["stay_duration"])
            }
            for point in self.recorded_points
        ]
        with open(file_path, 'w') as file:
            json.dump({'points': rounded_points}, file, indent=4)

    # 讀取地圖元資料
    def load_map_metadata(self, index):
        """根據選取的地圖讀取 YAML 檔案中的元資料"""
        try:
            with open(self.yaml_files[index], 'r') as file:
                map_info = yaml.safe_load(file)
            
            # 獲取解析度和原點座標
            self.resolution = map_info.get('resolution', 0.05)  # 如果缺少，設為默認值
            self.origin = map_info.get('origin', [0, 0, 0])  # 如果缺少，設為默認值
            print(f"地圖元資料加載成功: resolution={self.resolution}, origin={self.origin}")
        except Exception as e:
            print(f"加載地圖元資料失敗: {e}")

    
    def load_saved_points(self):
        """載入預設點位檔案"""
        # default_file_path = os.path.expanduser("C:/Users/ADMIN/OneDrive/gui_python/saved_points.json")
        default_file_path = os.path.join(self.gui_ws_path, "saved_points.json")
        try:
            with open(default_file_path, 'r') as file:
                data = json.load(file)
                self.recorded_points = data['points']
                print(f"已載入預設點位檔案: {default_file_path}")
        except FileNotFoundError:
            self.recorded_points = []
            print("未找到預設點位檔案")

    def load_image_height(self, pgm_file):
        """取得圖片的實際高度（以像素為單位）"""
        image = Image.open(pgm_file)
        return image.height

    def scan_map_files(self, map_directory):
        """掃描地圖目錄中的所有地圖檔案，並按字母順序排列"""
        yaml_files = []
        pgm_files = []

        if os.path.exists(map_directory):
            for file_name in sorted(os.listdir(map_directory)):  # 使用 sorted() 對檔案進行字母排序
                if file_name.endswith('.yaml'):
                    yaml_path = os.path.join(map_directory, file_name)
                    pgm_path = yaml_path.replace('.yaml', '.pgm')  # 確保有對應的 .pgm 檔案
                    if os.path.exists(pgm_path):
                        yaml_files.append(yaml_path)
                        pgm_files.append(pgm_path)

        return yaml_files, pgm_files

    def update_map(self, index):
        """更新顯示的地圖影像和元資料"""
        if index == 0:
            print("未選擇地圖")
            self.label.clear()
            self.label.set_file_name(None)  # 清空檔案名稱
            # 清空之前的點位與方向
            self.label.points.clear()
            self.label.directions.clear()
            self.save_points()
            return

        # 清空之前的點位與方向
        self.label.points.clear()
        self.label.directions.clear()
        self.save_points()
        
        # ...載入地圖和元資料邏輯...
        self.scale_label.setText(f"比例尺: 1 格 = {self.label.resolution:.2f} 米")

        selected_index = index - 1  # 因為選單中新增了一個空白選項
        self.load_map_metadata(selected_index)  # 加載 YAML 元資料
        selected_pgm_file = self.pgm_files[selected_index]
        selected_yaml_file = self.yaml_files[selected_index]

        # # 檢查地圖檔案是否存在
        # if not os.path.exists(selected_pgm_file):
        #     print(f"地圖檔案不存在: {selected_pgm_file}")
        #     self.label.clear()  # 清空地圖顯示
        #     return

        # 檢查地圖檔案是否存在
        if not os.path.exists(selected_pgm_file) or not os.path.exists(selected_yaml_file):
            print(f"地圖檔案不存在: {selected_pgm_file} 或 {selected_yaml_file}")
            return

        # 更新顯示的地圖
        self.label.update_image(selected_pgm_file, self.load_image_height(selected_pgm_file), self.origin, self.resolution)
        self.label.current_map_file = selected_pgm_file  # 設置當前地圖檔案
        print(f"地圖已成功加載: {selected_pgm_file}")
        self.recorded_points = []
        self.directions = []
        self.save_points()
        # 執行 ROS2 的 service call 來加載新地圖
        try:
            command = [
                "ros2", "service", "call", "/map_server/load_map", "nav2_msgs/srv/LoadMap",
                f"{{map_url: '{selected_yaml_file}'}}"
            ]
            subprocess.run(command, check=True)
            print(f"成功加載地圖: {selected_yaml_file}")
        except subprocess.CalledProcessError as e:
            print(f"加載地圖失敗: {e}")

    def check_terminal_status(self):
        """檢查終端機進程狀態，若已結束則重置按鈕"""
        if self.terminal_process and self.terminal_process.poll() is not None:
            print("終端機已關閉，重置鍵盤模式按鈕")
            self.terminal_process = None
            self.keyboard_mode_button.setText("KEYBOARD MODE\n啟用")
            self.keyboard_mode_button.setStyleSheet("background-color: gray; color: white; font-size: 16px;")

class MapLabel(QLabel):
    def __init__(self, parent):
        super().__init__(parent)
        self.origin = (0, 0)
        self.resolution = 1
        self.image_height = 1
        self.scale_factor = 1.0 # 初始比例設為 1.0
        self.points = [] # 儲存所有點擊的位置
        self.directions = [] # 儲存所有的方向箭頭
        self.current_file_name = None  # 紀錄當前檔案名稱
        self.current_map_file = None  # 儲存當前選擇的地圖檔案
        self.gui_ws_path = os.path.expanduser("/home/nvidia/workspace/Security_Robot_AI/gui_ws")
        os.makedirs(self.gui_ws_path, exist_ok=True)

    def update_image(self, pgm_file, image_height, origin, resolution, scale_factor=None):
        """更新顯示的地圖影像"""
        if scale_factor is not None:
            self.scale_factor = scale_factor

        # 防止多次進入
        if not pgm_file or not os.path.exists(pgm_file):
            print("無效地圖文件")
            return

        # 避免重複進行圖像縮放操作
        if hasattr(self, '_updating') and self._updating:
            print("正在更新圖片，忽略重複調用")
            return
        self._updating = True  # 標記開始更新
        self.image_height = image_height
        self.origin = origin
        self.resolution = resolution

        # 加載並縮放圖片
        qimage = QImage(pgm_file)
        if qimage.isNull():
            print(f"無法加載圖片：{pgm_file}")
            self._updating = False
            return

        scaled_width = int(qimage.width() * self.scale_factor)
        scaled_height = int(qimage.height() * self.scale_factor)

        pixmap = QPixmap(scaled_width, scaled_height)
        pixmap.fill(Qt.transparent)

        painter = QPainter(pixmap)
        painter.end()

        self.setPixmap(pixmap)
        self._updating = False
        print(f"更新圖片完成，縮放比例={self.scale_factor}")

    def update_points_and_directions(self, points):
        """根據檔案資訊更新地圖上的點與方向"""
        self.points.clear()
        self.directions.clear()

        for index, point in enumerate(points):
            try:
                # 確保所有必要的資料欄位存在
                required_fields = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
                if not all(field in point for field in required_fields):
                    print(f"點位資料不完整: {point}")
                    continue

                # 將世界座標轉換為像素座標
                pixel_pos = self.world_to_pixel(QPointF(point['x'], point['y']))
                self.points.append(pixel_pos)

                # 計算方向箭頭
                angle = self.quaternion_to_angle(
                    point['qx'], point['qy'], point['qz'], point['qw']
                )
                arrow_length = 50  # 固定箭頭長度
                direction_end = pixel_pos + QPointF(
                    arrow_length * math.cos(angle),
                    arrow_length * math.sin(angle)
                )
                self.directions.append((pixel_pos, direction_end))
                print(f"新增點位: {pixel_pos}，方向: {direction_end}")
            except Exception as e:
                print(f"處理點位失敗: {e}")

        self.update()  # 觸發重繪

    def world_to_pixel(self, world_pos):
        """將世界座標轉換為像素座標"""
        scaled_x = (world_pos.x() - self.origin[0]) / self.resolution * self.scale_factor
        scaled_y = (self.image_height - (world_pos.y() - self.origin[1]) / self.resolution) * self.scale_factor
        return QPointF(scaled_x, scaled_y)

    def pixel_to_world(self, pixel):
        """將像素座標轉換為世界座標"""
        scaled_px = pixel.x() / self.scale_factor
        scaled_py = pixel.y() / self.scale_factor
        world_x = self.origin[0] + scaled_px * self.resolution
        world_y = self.origin[1] + (self.image_height - scaled_py) * self.resolution
        print(f"計算的世界座標: ({world_x}, {world_y})")
        return world_x, world_y
    
    def world_points_from_pixels(self, pixel_points):
        """將像素座標列表轉換為世界座標列表"""
        world_points = []
        for pixel in pixel_points:
            # 使用 pixel_to_world 方法進行轉換
            world_x, world_y = self.pixel_to_world(pixel)
            world_points.append({"x": world_x, "y": world_y, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0})
        return world_points

    def clear(self):
        """清除顯示的地圖影像"""
        self.setPixmap(QPixmap())
        self.points.clear()
        self.directions.clear()
        self.update()

    def clear_map(self):
        """清除顯示的地圖影像"""
        self.setPixmap(QPixmap())  # 清除顯示
        self.points.clear()  # 清空點位
        self.directions.clear()  # 清空方向
        self.update()  # 更新顯示
        print("地圖已清空")

    def mousePressEvent(self, event: QMouseEvent):
        if self.window().recording_mode:
            # 呼叫自訂邏輯，顯示點擊的世界座標
            self.get_pixel_position(event)

            world_x, world_y = self.pixel_to_world(event.pos())

            if len(self.points) % 2 == 0:
                # 偶數次點擊：記錄點位
                self.points.append(event.pos() / self.scale_factor)  # 記錄縮放後的像素位置
                
                # 讀取停留時間
                stay_duration_hours = self.window().stay_duration_hours.value()
                stay_duration_minutes = self.window().stay_duration_minutes.value()
                stay_duration_seconds = self.window().stay_duration_seconds.value()
                stay_duration = (stay_duration_hours * 3600) + (stay_duration_minutes * 60) + stay_duration_seconds

                # 記錄世界座標和停留時間
                self.window().recorded_points.append({
                    "x": world_x,
                    "y": world_y,
                    "z": 0.0,
                    "qx": 0.0,
                    "qy": 0.0,
                    "qz": 0.0,
                    "qw": 1.0,
                    "stay_duration": stay_duration
                })
                print(f"記錄世界座標: ({world_x:.2f}, {world_y:.2f}), 停留時間: {stay_duration} 秒")
            
                # 同步更新目前檔案和 saved_points.json
                self.sync_points_to_files()
                
            else:
                # 奇數次點擊：計算方向
                if not self.points:
                    print("錯誤：尚未記錄任何點，無法設置方向")
                    return  # 避免 IndexError
                
                start_point = self.points[-1]
                end_point = event.pos() / self.scale_factor

                # 新增方向箭頭
                self.directions.append((start_point, end_point))
                self.points.pop()  # 移除最後一個點，避免箭頭終點出現紅點

                # 計算方向角度，並更新四元數
                angle = math.atan2(end_point.y() - start_point.y(), end_point.x() - start_point.x())
                qx, qy, qz, qw = calculate_quaternion_from_angle(angle)

                # 更新最後記錄的點位，添加方向信息
                last_point = self.window().recorded_points[-1]
                last_point.update({"qx": qx, "qy": qy, "qz": qz, "qw": qw})
                print(f"記錄方向: {last_point}")

            # 更新畫面
            self.update()
        
    def sync_points_to_files(self):
        """同步點位到 saved_points.json"""
        # saved_points_path = os.path.expanduser("C:/Users/ADMIN/OneDrive/gui_python/saved_points.json")
        saved_points_path = os.path.join(self.window().gui_ws_path, "saved_points.json")
        self.window().save_points_to_file(saved_points_path)
        print("已同步更新至 saved_points.json")

    def get_pixel_position(self, event):
        """在左下角顯示點擊的世界座標"""
        px = event.pos().x()
        py = event.pos().y()
        world_x, world_y = self.pixel_to_world(event.pos())
        print(f"點擊了像素位置: ({px}, {py}), 世界座標: ({world_x:.2f}, {world_y:.2f})")
        
        # 更新左下角顯示的座標
        self.window().value_label.setText(f"世界座標: ({world_x:.2f}, {world_y:.2f})")

    def paintEvent(self, event):
        # 打印當前縮放比例
        print('self.scale_factor=', self.scale_factor)
        
        # 調用父類的 paintEvent 方法以確保基礎繪製
        super().paintEvent(event)
        if not self.pixmap() or self.pixmap().isNull():
            print("無可顯示地圖")  # 確認無法繪製的情況
            return
        # 初始化 QPainter
        painter = QPainter(self)

        # 加載圖片（根據當前選擇的地圖）
        if self.current_map_file:
            qimage = QImage(self.current_map_file)
            if qimage.isNull():
                print(f"無法加載地圖圖片: {self.current_map_file}")
                return

            # 計算縮放後的寬度和高度
            w = int(qimage.size().width() * self.scale_factor)
            h = int(qimage.size().height() * self.scale_factor)

            # 繪製地圖圖片
            painter.drawImage(QRect(0, 0, w, h), qimage)
        else:
            print("未選擇地圖，無法繪製地圖影像")

        # 繪製輔助格線
        self.draw_grid(painter)

        # 繪製地圖原點
        self.draw_origin(painter)

        # 繪製所有點擊位置的點
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("red"))
        for i, point in enumerate(self.points):
            if i % 2 == 0:  # 只在每對點的起點繪製紅點
                adjusted_point = QPointF(point * self.scale_factor)  # 調整點位根據縮放
                painter.drawEllipse(adjusted_point, 5, 5)  # 5x5 像素的紅色圓點

        # 繪製所有方向箭頭
        for j, (start_point, end_point) in enumerate(self.directions):
            adjusted_start = QPointF(start_point * self.scale_factor)
            adjusted_end = QPointF(end_point * self.scale_factor)

            # 繪製起點的紅色圓點
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor("red"))
            painter.drawEllipse(adjusted_start, 5, 5)

            # 繪製箭頭線
            painter.setPen(QColor("blue"))
            painter.setBrush(Qt.NoBrush)
            arrow_length = 50  # 箭頭長度
            angle = math.atan2(adjusted_end.y() - adjusted_start.y(), adjusted_end.x() - adjusted_start.x())
            adjusted_arrow_end = adjusted_start + QPointF(
                arrow_length * math.cos(angle),
                arrow_length * math.sin(angle)
            )
            painter.drawLine(adjusted_start, adjusted_arrow_end)

            # 繪製箭頭頭部
            self.draw_arrow_head(painter, adjusted_start, adjusted_arrow_end)

            # 標註箭頭序號
            painter.setPen(Qt.black)
            painter.drawText(int(adjusted_start.x()) + 10, int(adjusted_start.y()) + 10, str(j + 1))

        # 繪製比例尺
        self.draw_scale(painter)

        # 結束繪製
        painter.end()

    def draw_scale(self, painter):
        """在右下角繪製比例尺"""
        grid_spacing_pixels = 20  # 每格的像素間距
        actual_distance = grid_spacing_pixels * self.resolution / self.scale_factor  # 實際距離（米）

        # 設置比例尺文字
        scale_text = f"1 格 = {actual_distance:.2f} 米"
        self.window().scale_label.setText(scale_text) #new***

        # 計算顯示位置（右下角）
        margin = 10  # 邊距
        x_pos = self.width() - margin - painter.fontMetrics().width(scale_text)
        y_pos = self.height() - margin

    def draw_points_from_file(self, points):
        """根據從檔案讀取的點，在地圖上繪製紅點、方向箭頭和順序編號"""
        # 清空內部結構
        self.points.clear()
        self.directions.clear()

        # 檢查地圖是否存在
        pixmap = self.pixmap()
        if pixmap is None:
            print("地圖尚未載入，無法繪製點位")
            return

        # 開始繪圖
        painter = QPainter(pixmap)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("red"))

        for index, point in enumerate(points):
            try:
                # 將世界座標轉為像素座標
                pixel_pos = self.world_to_pixel(QPointF(point['x'], point['y']))
                self.points.append(pixel_pos)  # 儲存點位

                # 畫紅點
                painter.drawEllipse(pixel_pos, 5, 5)

                # 如果有方向資訊，畫箭頭
                if 'qx' in point and 'qy' in point and 'qz' in point and 'qw' in point:
                    self.draw_arrow_from_orientation(
                        painter,
                        pixel_pos,
                        point['qx'],
                        point['qy'],
                        point['qz'],
                        point['qw']
                    )

                # 在紅點旁標註序號
                painter.setPen(Qt.black)
                painter.drawText(int(pixel_pos.x()) + 10, int(pixel_pos.y()) + 10, str(index + 1))
            except Exception as e:
                print(f"無法繪製點位: {point}，錯誤: {e}")

        painter.end()
        self.update()


    def draw_grid(self, painter):
        """在地圖上繪製輔助格線"""
        grid_spacing = 20 # 格線間距為20像素
        painter.setPen(QColor(200, 200, 200)) # 灰色的格線顏色

        # 繪製垂直格線
        for x in range(0, self.width(), grid_spacing):
            painter.drawLine(x, 0, x, self.height())

        # 繪製水平格線
        for y in range(0, self.height(), grid_spacing):
            painter.drawLine(0, y, self.width(), y)

    def set_file_name(self, file_name):
        """設定當前檔案名稱"""
        self.current_file_name = file_name if file_name else "無"
        self.window().file_name_label.setText(f"檔案名稱: {self.current_file_name}")
        self.update()  # 觸發重繪

    def draw_origin(self, painter):
        """繪製地圖原點和方向箭頭"""
        painter.setPen(QColor("green"))
        painter.setBrush(QColor("green"))

        # 計算 (0, 0) 點在地圖上的像素位置
        zero_px_x = (-self.origin[0]) / self.resolution * self.scale_factor
        zero_px_y = (self.image_height - (-self.origin[1]) / self.resolution) * self.scale_factor

        # 圓點的半徑
        radius = 5  # 設定為5像素半徑，總大小為10x10像素

        # 繪製圓點，確保圓心在原點位置
        painter.drawEllipse(int(zero_px_x) - radius, int(zero_px_y) - radius, radius * 2, radius * 2)

        # 繪製方向箭頭
        painter.setPen(QColor("blue"))
        painter.setBrush(Qt.NoBrush)

        arrow_length = 50
        angle = 0  # 假設箭頭指向正右方 (0 度)
        end_x = zero_px_x + arrow_length * math.cos(angle)
        end_y = zero_px_y + arrow_length * math.sin(angle)
        painter.drawLine(QPointF(zero_px_x, zero_px_y), QPointF(end_x, end_y))
        self.draw_arrow_head(painter, QPointF(zero_px_x, zero_px_y), QPointF(end_x, end_y))


    def draw_arrow_head(self, painter, start_point, end_point):
        """繪製方向箭頭"""
        arrow_size = 10 # 箭頭的大小
        angle = math.atan2(start_point.y() - end_point.y(), start_point.x() - end_point.x())

        # 箭頭的兩個邊角
        arrow_p1 = end_point + QPointF(arrow_size * math.cos(angle + math.pi / 6),
        arrow_size * math.sin(angle + math.pi / 6))
        arrow_p2 = end_point + QPointF(arrow_size * math.cos(angle - math.pi / 6),
        arrow_size * math.sin(angle - math.pi / 6))

        # 畫箭頭
        arrow_head = QPolygonF([end_point, arrow_p1, arrow_p2])
        painter.drawPolygon(arrow_head)

    def draw_arrow_from_orientation(self, painter, position, qx, qy, qz, qw):
        """根據四元數方向在地圖上繪製箭頭"""
        angle = quaternion_to_angle(qx, qy, qz, qw)  # 使用修正後的角度計算函數
        arrow_length = 50  # 箭頭的長度

        # 計算箭頭終點
        end_x = position.x() + arrow_length * math.cos(angle)
        end_y = position.y() + arrow_length * math.sin(angle)  # 修正 y 軸方向

        end_point = QPointF(end_x, end_y)

        # 畫箭頭主線
        painter.setPen(QColor("blue"))
        painter.drawLine(position, end_point)

        # 繪製箭頭頭部
        self.draw_arrow_head(painter, position, end_point)

    def resizeEvent(self, event):
        """在視窗大小改變時，自動重新縮放地圖"""
        if self.pixmap() and self.current_map_file:
            self.update_image(
                self.current_map_file,  # 傳入地圖檔案路徑
                self.image_height,
                self.origin,
                self.resolution,
                self.scale_factor
            )
        super().resizeEvent(event)

def calculate_quaternion_from_angle(angle):
    """根據 z 軸旋轉角度計算旋轉矩陣並轉換為四元數"""
    # 構建旋轉矩陣
    cos_theta = math.cos(angle)
    sin_theta = math.sin(angle)
    
    # 旋轉矩陣
    R = [
        [cos_theta, -sin_theta, 0],
        [sin_theta, cos_theta, 0],
        [0, 0, 1]
    ]
    
    # 計算四元數
    qw = math.sqrt(1 + R[0][0] + R[1][1] + R[2][2]) / 2
    
    # 添加保護機制，避免除以零
    if qw == 0:
        qw = 1e-8  # 給予一個非常小的值，避免除零錯誤
    
    qz = -((R[1][0] - R[0][1]) / (4 * qw))
    
    return 0.0, 0.0, qz, qw

def quaternion_to_angle(qx, qy, qz, qw):
    """
    根據四元數計算旋轉角度（僅限於 z 軸旋轉），並修正 PyQt 的座標系反向。
    """
    angle = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))
    return -angle  # 修正 PyQt 坐標系


if __name__ == '__main__':
    # 地圖檔案路徑
    yaml_files = []
    pgm_files = []

    app = QApplication(sys.argv)
    window = MapWindow(yaml_files, pgm_files)
    window.show()
    sys.exit(app.exec_())