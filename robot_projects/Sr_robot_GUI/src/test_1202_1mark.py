import sys
import yaml
import json
import math
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QComboBox, QInputDialog, QFileDialog, QMessageBox
from PyQt5.QtGui import QPixmap, QPainter, QColor, QMouseEvent, QPolygonF,QImage
from PyQt5.QtCore import Qt, pyqtSlot, QPointF,QRect
from PIL import Image
import subprocess
from PyQt5.QtWidgets import QSpinBox ,QScrollArea
import os



class MapWindow(QMainWindow):
    def __init__(self, yaml_files, pgm_files):
        super().__init__()
        self.terminal_process = None  # 用於保存終端機進程
        self.file_loaded = False  # 紀錄是否載入過檔案
        

        # 設定地圖目錄
        self.map_directory = "/home/sr/wheeltec_ros2/src/wheeltec_robot_nav2/map"
        #"/home/sr/wheeltec_ros2/src/wheeltec_robot_nav2/map"
        # 自動掃描目錄中的地圖檔案
        self.yaml_files, self.pgm_files = self.scan_map_files(self.map_directory)
        # 程式啟動時自動執行指令
        #self.start_ros2_process()#################
        # 初始化時載入已儲存的點位和方向
        self.load_saved_points()
        # 設定視窗標題
        self.setWindowTitle('Map with Control Buttons, Image Selector, and Grid')
        self.setFixedSize(1200, 700) # 設定固定視窗大小
        # 設定主視窗佈局
        main_layout = QVBoxLayout()

        # 建立下拉式選單，顯示地圖名稱
        self.combo_box = QComboBox(self)
        self.combo_box.addItem("Select a map")  # 新增空白選項
        self.combo_box.addItems([os.path.splitext(os.path.basename(yaml))[0] for yaml in self.yaml_files])  # 只顯示檔名
        self.combo_box.currentIndexChanged.connect(self.update_map)  # 連接選購變更事件
        main_layout.addWidget(self.combo_box)

        # 建立地圖顯示區域
        self.label = MapLabel(self)  # 初始化地圖顯示區域
        self.scroll_area = QScrollArea(self)  # 創建滾動區域
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setWidget(self.label)  # 設置地圖為滾動區域的子元件

        # 建立地圖佈局
        map_layout = QVBoxLayout()
        map_layout.addWidget(self.scroll_area)  # 添加滾動區域
        map_layout.setContentsMargins(0, 0, 0, 0)


        # 建立按鈕並設定其樣式
        self.start_button = QPushButton("START\n啟用")
        self.stop_button = QPushButton("STOP\n停止")
        self.home_button = QPushButton("HOME\n回原點")
        self.set_point_button = QPushButton("SET POINT\n設定點位")
        self.clear_button = QPushButton("CLEAR\n清除點位")
        self.zoom_in_button = QPushButton("Zoom In\n放大")
        self.zoom_out_button = QPushButton("Zoom Out\n縮小")
        self.save_file_button = QPushButton("SAVE FILE\n儲存檔案")
        self.delete_file_button = QPushButton("DELETE FILE\n刪除檔案")
        self.load_file_button = QPushButton("LOAD FILE\n載入檔案")
        self.cancel_file_button = QPushButton("CANCEL FILE\n取消檔案")
        
        # 綁定按鈕事件
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
        

        # 設定按鈕大小和顏色
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

        # 新增水平佈局，用於放置儲存檔案和刪除檔案按鈕
        file_buttons_layout1 = QHBoxLayout()
        file_buttons_layout1.addWidget(self.set_point_button)
        file_buttons_layout1.addWidget(self.zoom_in_button)

        # 新增水平佈局，用於放置載入檔案和取消檔案按鈕
        file_buttons_layout2 = QHBoxLayout()
        file_buttons_layout2.addWidget(self.clear_button)
        file_buttons_layout2.addWidget(self.zoom_out_button)
        
        # 新增水平佈局，用於放置儲存檔案和刪除檔案按鈕
        file_buttons_layout3 = QHBoxLayout()
        file_buttons_layout3.addWidget(self.save_file_button)
        file_buttons_layout3.addWidget(self.delete_file_button)

        # 新增水平佈局，用於放置載入檔案和取消檔案按鈕
        file_buttons_layout4 = QHBoxLayout()
        file_buttons_layout4.addWidget(self.load_file_button)
        file_buttons_layout4.addWidget(self.cancel_file_button)

        # 建立按鈕佈局
        button_layout = QVBoxLayout()
        button_layout.setContentsMargins(5, 5, 5, 5)
        button_layout.setSpacing(20)
        button_layout.addWidget(self.start_button)
        button_layout.addWidget(self.stop_button)
        button_layout.addWidget(self.home_button)
        button_layout.addStretch() # 增加伸縮空間，確保按鈕在頂部
        # button_layout.addWidget(self.load_file_button)
        button_layout.addLayout(file_buttons_layout1)
        button_layout.addLayout(file_buttons_layout2)
        button_layout.addLayout(file_buttons_layout3)
        button_layout.addLayout(file_buttons_layout4)

        

        # 建立地圖顯示區域
        self.label = MapLabel(self) # 初始時不顯示地圖
        #self.label.setAlignment(Qt.AlignCenter)###########mark

        # 建立數值顯示區域
        self.value_label = QLabel(self)
        self.value_label.setFixedSize(150, 30)
        self.value_label.setStyleSheet("background-color: white; border: 1px solid black;")

        # 建立地圖佈局
        map_layout = QVBoxLayout()
        map_layout.addWidget(self.label)
        map_layout.setContentsMargins(0, 0, 0, 0)

        # 將按鈕佈局和地圖佈局新增至主佈局
        content_layout = QHBoxLayout()
        content_layout.addLayout(button_layout)
        content_layout.addLayout(map_layout)
        main_layout.addLayout(content_layout)
        main_layout.addWidget(self.value_label)

        # 設定中心視窗
        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        # 記錄模式狀態
        self.recording_mode = False
        self.current_waypoint = None


        # 建立停留時間設定區域 (小時、分鐘、秒)
        self.stay_duration_label = QLabel('停留時間 (小時:分鐘:秒)', self)
        self.stay_duration_label.setFixedSize(150, 50)

        self.stay_duration_hours = QSpinBox(self)
        self.stay_duration_hours.setRange(0, 23)  # 小時範圍 0 到 23
        self.stay_duration_hours.setValue(0)  # 預設為 0 小時
        self.stay_duration_hours.setFixedSize(60, 50)

        self.stay_duration_minutes = QSpinBox(self)
        self.stay_duration_minutes.setRange(0, 59)  # 分鐘範圍 0 到 59
        self.stay_duration_minutes.setValue(0)  # 預設為 0 分鐘
        self.stay_duration_minutes.setFixedSize(60, 50)

        self.stay_duration_seconds = QSpinBox(self)
        self.stay_duration_seconds.setRange(0, 59)  # 秒範圍 0 到 59
        self.stay_duration_seconds.setValue(10)  # 預設為 10 秒
        self.stay_duration_seconds.setFixedSize(60, 50)

        # 建立一個橫向佈局來放置時間控制元件
        time_layout = QHBoxLayout()
        time_layout.addWidget(self.stay_duration_hours)
        time_layout.addWidget(self.stay_duration_minutes)
        time_layout.addWidget(self.stay_duration_seconds)

        # 將停留時間標籤和時間控制元件添加到按鈕佈局中
        button_layout.addWidget(self.stay_duration_label)
        button_layout.addLayout(time_layout)

    def closeEvent(self, event):
        """當視窗關閉時，觸發停止導航"""
        print("視窗關閉事件觸發，執行停止操作")
        #subprocess.Popen([
        #    "xterm", "-e", "bash -c 'pkill -f ros2; exec bash'"
        #])# 在關閉視窗時自動停止導航
        event.accept()  # 接受並繼續關閉視窗
    
    def save_file_with_name(self):
        """儲存當前點位到檔案，將地圖名稱添加到檔案名稱前"""
        # 獲取當前地圖名稱
        current_map_name = self.combo_box.currentText()
        if current_map_name == "Select a map":
            QMessageBox.warning(self, "警告", "請先選擇一個地圖再進行儲存操作！")
            return

        if self.file_loaded and self.label.current_file_name:  # 已載入檔案且有檔案名稱
            # 使用 QMessageBox 確認覆蓋或另存新檔案
            msg_box = QMessageBox(self)
            msg_box.setWindowTitle("儲存檔案")
            msg_box.setText(f"已載入檔案：{self.label.current_file_name}\n您想要覆蓋此檔案嗎？")
            msg_box.setStandardButtons(QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel)
            msg_box.setDefaultButton(QMessageBox.Yes)
            response = msg_box.exec_()

            if response == QMessageBox.Yes:  # 覆蓋原檔案
                save_path = os.path.expanduser(f"~/gui_ws/{current_map_name}_{self.label.current_file_name}")
                self.save_points_to_file(save_path)
                print(f"已覆蓋原檔案：{save_path}")
            elif response == QMessageBox.No:  # 另存為新檔案
                file_name, ok = QInputDialog.getText(self, "另存為", "請輸入新檔案名稱（不含副檔名）:")
                if ok and file_name:
                    save_path = os.path.expanduser(f"~/gui_ws/{current_map_name}_{file_name}.json")
                    self.save_points_to_file(save_path)
                    print(f"已儲存至新檔案：{save_path}")
            else:
                print("取消儲存操作")
        else:
            # 未載入檔案，直接要求輸入新檔案名稱
            file_name, ok = QInputDialog.getText(self, "儲存檔案", "請輸入檔案名稱（不含副檔名）:")
            if ok and file_name:
                save_path = os.path.expanduser(f"~/gui_ws/{current_map_name}_{file_name}.json")
                self.save_points_to_file(save_path)
                print(f"點位已儲存至: {save_path}")
            else:
                print("取消儲存操作")



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

    def zoom_in(self):
        """放大地圖"""
        self.label.scale_factor =self.label.scale_factor * 1.2  # 每次放大 20%
        self.label.update()
        #self.label.paintEvent(QMouseEvent)
        #self.label.update_image(self.label.pixmap().toImage(), self.label.image_height, self.label.origin, self.label.resolution,self.label.scale_factor)#mark

    def zoom_out(self):
        """縮小地圖"""
        self.label.scale_factor = self.label.scale_factor /1.2  # 每次縮小 20%############
        self.label.update()
        #self.label.paintEvent(QMouseEvent)
        #self.label.update_image(self.label.pixmap().toImage(), self.label.image_height, self.label.origin, self.label.resolution,self.label.scale_factor)#mark


    def load_file(self):
        """載入特定檔案並根據檔案名稱自動切換地圖"""
        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getOpenFileName(
            self, "選擇檔案", os.path.expanduser("~/gui_ws"), "JSON Files (*.json)", options=options
        )
        if not file_path:  # 使用者未選擇檔案
            print("未選擇檔案")
            return

        try:
            with open(file_path, 'r') as file:
                data = json.load(file)

            # 驗證檔案結構
            if 'points' not in data or not isinstance(data['points'], list):
                raise ValueError("JSON 檔案格式不正確，缺少 'points' 欄位或結構不符")

            # 從檔案名稱中提取地圖名稱
            file_name = os.path.basename(file_path)
            map_name = file_name.split('_')[0]  # 假設檔案名稱格式為 mapname_xxx.json
            print(f"從檔案名稱提取的地圖名稱: {map_name}")

            # 在下拉選單中自動選擇對應地圖
            index = self.combo_box.findText(map_name)
            if index == -1:
                QMessageBox.warning(self, "警告", f"未找到與地圖名稱 '{map_name}' 匹配的地圖")
                return
            self.combo_box.setCurrentIndex(index)

            # 調用地圖更新邏輯
            self.update_map(index)

            # 清空之前的點位與方向
            self.label.points.clear()
            self.label.directions.clear()

            # 更新 MapLabel 的檔案名稱
            self.label.set_file_name(file_name)

            # 解析檔案中的點位與方向數據
            for point in data['points']:
                # 驗證點位是否完整
                if not all(key in point for key in ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']):
                    print(f"點位數據不完整，跳過: {point}")
                    continue

                # 計算像素位置
                pixel_pos = self.label.world_to_pixel(QPointF(point['x'], point['y']))
                self.label.points.append(pixel_pos)

                # 更新 recorded_points 並設置 file_loaded 為 True
                self.recorded_points = data['points']
                self.file_loaded = True

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

            print(f"成功載入檔案: {file_path}")
            self.label.update()  # 觸發重新繪製
        except Exception as e:
            print(f"載入檔案失敗: {e}")



    def start_ros2_process(self):
        """啟動 ROS2 指令"""
        print("啟動 ROS2 進程")

        try:
            # 組合要執行的指令
            command = "cd ~/wheeltec_ros2 && source install/setup.bash && ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py"
            
            # 使用 subprocess.Popen 來啟動 ROS2 進程，這樣不會阻塞主程式
            subprocess.Popen(f"bash -c '{command}'", shell=True, executable='/bin/bash')

            print("ROS2 進程已成功啟動")
        except subprocess.CalledProcessError as e:
            print(f"啟動過程失敗: {e}")


    def load_map_metadata(self, index):
        """根據選取的地圖讀取 YAML 檔案中的元資料"""
        with open(self.yaml_files[index], 'r') as file:
            map_info = yaml.safe_load(file)

        self.resolution = map_info['resolution']
        self.origin = map_info['origin']

    @pyqtSlot()
    def start_process(self):
        """啟動導航進程"""
        if self.terminal_process is None or self.terminal_process.poll() is not None:
            # 啟動前更新 JSON 檔案
            default_json_path = "/home/sr/gui_ws/saved_points.json"
            self.save_points_to_file(default_json_path)
            print(f"已更新導航點位檔案至: {default_json_path}")
            
            # 啟動導航進程
            self.terminal_process = subprocess.Popen([
                "xterm", "-e", "bash -c 'cd ~/wheeltec_ros2 && source install/setup.bash && "
                "/bin/python3.10 /home/sr/wheeltec_ros2/src/wheeltec_robot_nav2/launch/waypoint_testgui_time.py; exec bash'"
            ])
            print(f"啟動的終端機進程 PID: {self.terminal_process.pid}")
        else:
            print("終端機進程已經在運行，無法再次啟動")

    @pyqtSlot()
    def stop_process(self):
        if self.terminal_process:
            # 結束啟動的單一終端機進程
            print(f"停止終端機進程 PID: {self.terminal_process.pid}")
            self.terminal_process.terminate()  # 只結束該特定進程
            self.terminal_process.wait()  # 等待進程完全終止
            self.terminal_process = None  # 重置以允許再次啟動
        print("導航已停止")

    @pyqtSlot()
    def home_process(self):
        # 清除之前的所有點位，並儲存原點作為唯一導航點
        print("回原點按鈕被點擊，清除所有點位並儲存原點")
        
        # 建立原點點位
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
        
        # 清除之前的點位
        self.recorded_points = [origin_waypoint]  # 只儲存原點為唯一點位
        
        # 儲存更新後的點位到 saved_points.json
        self.save_points()

        # 顯示成功訊息
        print("已清除所有點位並將原點儲存為新點位")




    @pyqtSlot()
    def toggle_set_point_mode(self):
        """切換點位記錄模式"""
        self.recording_mode = not self.recording_mode
        if self.recording_mode:
            self.set_point_button.setStyleSheet("background-color: orange; color: white; font-size: 18px;")
            # 若已載入檔案，不清空點位
            if self.file_loaded:
                print("已進入點位記錄模式（保留載入的點位）")
            else:
                # 若未載入檔案，清空點位
                self.recorded_points = []
                print("已進入點位記錄模式（未載入檔案，清空點位）")
        else:
            self.set_point_button.setStyleSheet("background-color: gray; color: white; font-size: 18px;")
            self.save_points()  # 儲存記錄的點位
            print("已退出點位記錄模式")


    @pyqtSlot()
    def clear_points(self):
        """清除所有記錄的點位"""
        self.recorded_points = []
        self.directions = []
        self.label.clear()
        print("所有記錄的點位已清除")

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
        with open('/home/sr/gui_ws/saved_points.json', 'w') as file:
            json.dump({'points': rounded_points}, file, indent=4)

        print("已將點位儲存到 saved_points.json")
    
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

        try:
            with open(file_path, 'w') as file:
                json.dump({'points': rounded_points}, file, indent=4)
            print(f"成功儲存點位至: {file_path}")
        except Exception as e:
            print(f"儲存失敗: {e}")

    def delete_file(self):
        """刪除選擇的 JSON 檔案"""
        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getOpenFileName(
            self, "選擇要刪除的檔案", os.path.expanduser("~/gui_ws"), "JSON Files (*.json)", options=options
        )
        if not file_path:  # 如果未選擇檔案
            print("未選擇檔案")
            return

        # 使用 QMessageBox 確認刪除
        msg_box = QMessageBox(self)
        msg_box.setWindowTitle("確認刪除")
        msg_box.setText(f"是否確定要刪除檔案：{os.path.basename(file_path)}？")
        msg_box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        msg_box.setDefaultButton(QMessageBox.No)
        response = msg_box.exec_()

        if response == QMessageBox.Yes:
            try:
                os.remove(file_path)
                print(f"成功刪除檔案: {file_path}")
            except Exception as e:
                print(f"刪除檔案失敗: {e}")
        else:
            print("取消刪除操作")



    def load_saved_points(self):
        """載入預設點位檔案"""
        default_file_path = os.path.expanduser("~/gui_ws/saved_points.json")
        try:
            with open(default_file_path, 'r') as file:
                data = json.load(file)
                self.recorded_points = data['points']
                print(f"已載入預設點位檔案: {default_file_path}")
        except FileNotFoundError:
            self.recorded_points = []
            print("未找到預設點位檔案")

    def cancel_file(self):
        """取消已載入的檔案並清空點位，同時重置地圖顯示"""
        if not self.file_loaded:
            print("尚未載入任何檔案，無需取消")
            return

        # 重置檔案載入狀態
        self.file_loaded = False
        self.recorded_points = []  # 清空點位
        self.label.points.clear()
        self.label.directions.clear()

        # 清空檔案名稱
        self.label.set_file_name(None)  # 清除檔案名稱

        # 重置地圖顯示
        self.combo_box.setCurrentIndex(0)  # 設置為 "Select a map"
        self.label.clear_map()  # 隱藏地圖
        print("已取消載入的檔案，所有點位及檔案名稱已清空")




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

    # def update_map(self, index):
    #     """更新顯示的地圖影像和元資料"""
    #     if index == 0:
    #         # 空白選項，隱藏地圖
    #         self.label.clear()
    #         return

    #     selected_index = index - 1  # 因為選單中新增了一個空白選項
    #     self.load_map_metadata(selected_index)
    #     selected_pgm_file = self.pgm_files[selected_index]
    #     selected_yaml_file = self.yaml_files[selected_index]

    #     # 檢查地圖檔案是否存在
    #     if not os.path.exists(selected_pgm_file) or not os.path.exists(selected_yaml_file):
    #         print(f"地圖檔案不存在: {selected_pgm_file} 或 {selected_yaml_file}")
    #         return

    #     # 更新顯示的地圖
    #     self.label.update_image(selected_pgm_file, self.load_image_height(selected_pgm_file), self.origin, self.resolution)

    #     # 執行 ROS2 的 service call 來加載新地圖
    #     try:
    #         command = [
    #             "ros2", "service", "call", "/map_server/load_map", "nav2_msgs/srv/LoadMap",
    #             f"{{map_url: '{selected_yaml_file}'}}"
    #         ]
    #         subprocess.run(command, check=True)
    #         print(f"成功加載地圖: {selected_yaml_file}")
    #     except subprocess.CalledProcessError as e:
    #         print(f"加載地圖失敗: {e}")

    def update_map(self, index):
        """更新顯示的地圖影像和元資料"""
        if index == 0:
            # 空白選項，隱藏地圖
            self.label.clear()
            return

        selected_index = index - 1  # 因為選單中新增了一個空白選項
        self.load_map_metadata(selected_index)
        selected_pgm_file = self.pgm_files[selected_index]
        selected_yaml_file = self.yaml_files[selected_index]

        # 檢查地圖檔案是否存在
        if not os.path.exists(selected_pgm_file) or not os.path.exists(selected_yaml_file):
            print(f"地圖檔案不存在: {selected_pgm_file} 或 {selected_yaml_file}")
            return

        # 更新顯示的地圖
        self.label.update_image(selected_pgm_file, self.load_image_height(selected_pgm_file), self.origin, self.resolution)
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
        #super().paintEvent(event)

    def update_image(self, pgm_file, image_height, origin, resolution,scale_factor=1):
        """更新顯示的地圖影像，並根據縮放比例重新繪製"""
        self.image_height = image_height
        self.origin = origin
        self.resolution = resolution
        #pixmap = QPixmap(pgm_file)

        # 根據縮放比例調整地圖顯示
        #scaled_pixmap = pixmap.scaled(pixmap.size() * scale_factor,Qt.KeepAspectRatio,Qt.SmoothTransformation)
        #('pixmap.size()=',pixmap.size())
        #self.setPixmap(scaled_pixmap)
        qimage = QImage('WHEELTEC.pgm')
        print(pgm_file)
        w = int(qimage.size().width())
        h = int(qimage.size().height())
        print(w,h)
        self.painter.drawImage(QRect(0, 0, w, h), qimage)
        print('update')
        self.painter.end()
        #self.update()


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





    # def world_to_pixel(self, world_pos):
    #     """將世界座標轉換為像素座標"""
    #     scaled_x = (world_pos.x() - self.origin[0]) / self.resolution * self.scale_factor
    #     scaled_y = (self.image_height - (world_pos.y() - self.origin[1]) / self.resolution) * self.scale_factor
    #     return QPointF(scaled_x, scaled_y)

    # def quaternion_to_angle(self, qx, qy, qz, qw):
    #     """根據四元數計算旋轉角度（僅限於 z 軸旋轉）"""
    #     angle = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))
    #     return angle

    def world_to_pixel(self, world_pos):
        """將世界座標(M)轉換為像素座標(pix)"""
        scaled_x = (world_pos.x() - self.origin[0]) / self.resolution * self.scale_factor
        scaled_y = (self.image_height - (world_pos.y() - self.origin[1]) / self.resolution) * self.scale_factor
        return QPointF(scaled_x, scaled_y)

    def pixel_to_world(self, pixel):
        """將像素座標(pix)轉換為世界座標(M)"""
        scaled_px = pixel.x() / self.scale_factor
        scaled_py = pixel.y() / self.scale_factor
        world_x = self.origin[0] + scaled_px * self.resolution
        world_y = self.origin[1] + (self.image_height - scaled_py) * self.resolution
        return world_x, world_y


    def clear(self):
        """清除顯示的地圖影像"""
        # self.setPixmap(QPixmap())
        self.points.clear()
        self.directions.clear()
        self.update()

    def clear_map(self):
        """清除顯示的地圖影像"""
        self.setPixmap(QPixmap())
        self.update()




    def mousePressEvent(self, event: QMouseEvent):
        if self.window().recording_mode:
            # 呼叫自訂邏輯
            self.get_pixel_position(event)

            if len(self.points) % 2 == 0:
                # 偶數次點擊，新增點
                #self.points.append(event.pos())#記錄滑鼠點的pix位置
                #world_x, world_y = self.pixel_to_world(event.pos())
                self.points.append(event.pos()/self.scale_factor)#記錄滑鼠點的pix位置
                world_x, world_y = self.pixel_to_world(event.pos()/self.scale_factor)
                # 從小時、分鐘和秒的 SpinBox 讀取數值
                stay_duration_hours = self.window().stay_duration_hours.value()
                stay_duration_minutes = self.window().stay_duration_minutes.value()
                stay_duration_seconds = self.window().stay_duration_seconds.value()

                # 將小時、分鐘和秒轉換為總秒數
                stay_duration = (stay_duration_hours * 3600) + (stay_duration_minutes * 60) + stay_duration_seconds

                # 確認已經正確獲取停留時間
                # print(f"記錄世界點位: ({world_x}, {world_y}), 停留時間: {stay_duration} 秒")

                self.window().recorded_points.append({"x": world_x, "y": world_y, "z": 0.0, "stay_duration": stay_duration})
                print(f"記錄世界點位: ({world_x}, {world_y}), 停留時間: {stay_duration} 秒")
            else:
                # 奇數次點擊，計算方向並新增方向箭頭
                start_point = self.points[-1]
                #end_point = event.pos()
                end_point = event.pos()/self.scale_factor
                self.directions.append((start_point, end_point))
                self.points.pop()  # 移除最後一個點（第二次點擊不顯示紅點）
                angle = math.atan2(end_point.y() - start_point.y(), end_point.x() - start_point.x())
                qx, qy, qz, qw = calculate_quaternion_from_angle(angle)
                last_point = self.window().recorded_points[-1]
                last_point.update({"qx": qx, "qy": qy, "qz": qz, "qw": qw})
                print(f"記錄方向: {last_point}")

            self.update()




    def pixel_to_world(self, pixel):
        """將像素座標轉換為世界座標"""
        scaled_px = pixel.x() / self.scale_factor
        scaled_py = pixel.y() / self.scale_factor
        world_x = self.origin[0] + scaled_px * self.resolution
        world_y = self.origin[1] + (self.image_height - scaled_py) * self.resolution
        return world_x, world_y

    # def get_pixel_position(self, event):
    #     px = event.pos().x()
    #     py = event.pos().y()
    #     print(f"點擊了像素位置: ({px}, {py})")

    def get_pixel_position(self, event):
        """在左下角顯示點擊的世界座標"""
        px = event.pos().x()
        py = event.pos().y()
        world_x, world_y = self.pixel_to_world(event.pos())
        print(f"點擊了像素位置: ({px}, {py}), 世界座標: ({world_x:.2f}, {world_y:.2f})")
        
        # 更新左下角顯示的座標
        self.window().value_label.setText(f"世界座標: ({world_x:.2f}, {world_y:.2f})")


    def paintEvent(self, event):
        print('self.scale_factor=',self.scale_factor)
        super().paintEvent(event)
        painter = QPainter(self)
        qimage = QImage('WHEELTEC.pgm')
        w = int(qimage.size().width()*self.scale_factor)
        h = int(qimage.size().height()*self.scale_factor)
        #print(w,h)
        painter.drawImage(QRect(0, 0, w, h), qimage)
        print('update')
        # 繪製輔助格線
        self.draw_grid(painter)

        # 繪製地圖原點
        self.draw_origin(painter)

        # 繪製當前檔案名稱
        if self.current_file_name:
            painter.setPen(Qt.black)
            painter.drawText(10, 30, f"檔案名稱: {self.current_file_name}")

        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("red"))

        # 繪製所有點擊位置的點
        for i, point in enumerate(self.points):
            if i % 2 == 0:  # 只在每對點的起點繪製紅色圓點
                painter.drawEllipse(QPointF(point*self.scale_factor), 5, 5)  # 5x5 像素的紅色圓點

        # 繪製所有方向
        for j, (start_point, end_point) in enumerate(self.directions):
            start_point=start_point*self.scale_factor
            end_point=end_point*self.scale_factor
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor("red"))
            painter.drawEllipse(start_point, 5, 5)  # 5x5 像素的紅色圓點

            # 繪製箭頭
            painter.setPen(QColor("blue"))
            painter.setBrush(Qt.NoBrush)

            arrow_length = 50
            angle = math.atan2(end_point.y() - start_point.y(), end_point.x() - start_point.x())
            adjusted_end = start_point + QPointF(
                arrow_length * math.cos(angle),
                arrow_length * math.sin(angle)
            )
            painter.drawLine(start_point, adjusted_end)
            self.draw_arrow_head(painter, start_point, adjusted_end)

            # 標註箭頭序號
            painter.setPen(Qt.black)
            painter.drawText(int(start_point.x()) + 10, int(start_point.y()) + 10, str(j + 1))
        
        # 繪製比例尺
        self.draw_scale(painter)

        
    def draw_scale(self, painter):
        """在右下角繪製比例尺"""
        grid_spacing_pixels = 20  # 每格的像素間距
        actual_distance = grid_spacing_pixels * self.resolution / self.scale_factor  # 實際距離（米）

        # 設置比例尺文字
        scale_text = f"1 格 = {actual_distance:.2f} 米"

        # 計算顯示位置（右下角）
        margin = 10  # 邊距
        x_pos = self.width() - margin - painter.fontMetrics().width(scale_text)
        y_pos = self.height() - margin

        # 繪製比例尺文字
        painter.setPen(Qt.black)
        painter.drawText(x_pos, y_pos, scale_text)



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
        self.current_file_name = file_name if file_name else ""
        self.update()  # 觸發重繪


    def draw_origin(self, painter):
        """繪製地圖原點和方向箭頭"""
        painter.setPen(QColor("green"))
        painter.setBrush(QColor("green"))

        # 計算 (0, 0) 點在地圖上的像素位置
        zero_px_x = (-self.origin[0]) / self.resolution * self.scale_factor
        zero_px_y = (self.image_height - (-self.origin[1]) / self.resolution) * self.scale_factor
        #zero_px_y = self.image_height - ( (-self.origin[1]) / self.resolution * self.scale_factor)

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
        if self.pixmap():
            self.update_image(self.pixmap().toImage(), self.image_height, self.origin, self.resolution)
            super().resizeEvent(event)

    # def calculate_quaternion_from_angle(angle):
    #     """根據給定的角度計算四元數"""
    #     qw = math.cos(angle / 2)
    #     qz = math.sin(angle / 2)
    #     return 0.0, 0.0, qz, qw # 假設旋轉只發生在 z 軸

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