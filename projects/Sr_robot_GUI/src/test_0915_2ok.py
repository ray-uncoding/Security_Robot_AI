import sys
import yaml
import json
import math
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QComboBox
from PyQt5.QtGui import QPixmap, QPainter, QColor, QMouseEvent, QPolygonF
from PyQt5.QtCore import Qt, pyqtSlot, QPointF
from PIL import Image
import subprocess

class MapWindow(QMainWindow):
    def __init__(self, yaml_files, pgm_files):
        super().__init__()

        # 儲存 YAML 和 PGM 檔案路徑
        self.yaml_files = yaml_files
        self.pgm_files = pgm_files

        # 初始化時載入已儲存的點位和方向
        # self.load_saved_points()

        # 設定視窗標題
        self.setWindowTitle('Map with Control Buttons, Image Selector, and Grid')
        self.setFixedSize(1300, 1000) # 設定固定視窗大小

        # 設定主視窗佈局
        main_layout = QVBoxLayout()

        # 建立下拉式選單
        self.combo_box = QComboBox(self)
        self.combo_box.addItem("Select a map") # 新增空白選項
        self.combo_box.addItems([f"Map {i + 1}" for i in range(len(pgm_files))])
        self.combo_box.currentIndexChanged.connect(self.update_map) # 連接選購變更事件
        main_layout.addWidget(self.combo_box)

        # 建立按鈕並設定其樣式
        self.start_button = QPushButton("START\n啟用")
        self.stop_button = QPushButton("STOP\n停止")
        self.home_button = QPushButton("HOME\n回原點")
        self.set_point_button = QPushButton("SET POINT\n設定點位")
        self.clear_button = QPushButton("CLEAR\n清除點位")

        # 綁定按鈕事件
        self.start_button.clicked.connect(self.start_process)
        self.stop_button.clicked.connect(self.stop_process)
        self.home_button.clicked.connect(self.home_process)
        self.set_point_button.clicked.connect(self.toggle_set_point_mode)
        self.clear_button.clicked.connect(self.clear_points)

        # 設定按鈕大小和顏色
        self.start_button.setFixedSize(120, 150)
        self.start_button.setStyleSheet("background-color: green; color: white; font-size: 18px;")
        self.stop_button.setFixedSize(120, 150)
        self.stop_button.setStyleSheet("background-color: red; color: white; font-size: 18px;")
        self.home_button.setFixedSize(120, 150)
        self.home_button.setStyleSheet("background-color: blue; color: white; font-size: 18px;")
        self.set_point_button.setFixedSize(120, 150)
        self.set_point_button.setStyleSheet("background-color: gray; color: white; font-size: 18px;")
        self.clear_button.setFixedSize(120, 150)
        self.clear_button.setStyleSheet("background-color: orange; color: white; font-size: 18px;")

        # 建立按鈕佈局
        button_layout = QVBoxLayout()
        button_layout.setContentsMargins(20, 20, 20, 20)
        button_layout.setSpacing(20)
        button_layout.addWidget(self.start_button)
        button_layout.addWidget(self.stop_button)
        button_layout.addWidget(self.home_button)
        button_layout.addWidget(self.set_point_button)
        button_layout.addWidget(self.clear_button)
        button_layout.addStretch() # 增加伸縮空間，確保按鈕在頂部

        # 建立地圖顯示區域
        self.label = MapLabel(self) # 初始時不顯示地圖

        # 建立數值顯示區域
        self.value_label = QLabel(self)
        self.value_label.setFixedSize(200, 50)
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

    def load_map_metadata(self, index):
        """根據選取的地圖讀取 YAML 檔案中的元資料"""
        with open(self.yaml_files[index], 'r') as file:
            map_info = yaml.safe_load(file)

        self.resolution = map_info['resolution']
        self.origin = map_info['origin']

    @pyqtSlot()
    def start_process(self):
        print("啟動按鈕被點擊")
        subprocess.Popen([
            "xterm", "-e", "bash -c 'cd ~/wheeltec_ros2 && source install/setup.bash && ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py; exec bash'"
        ])

    @pyqtSlot()
    def stop_process(self):
        print("停止按鈕被點擊")
        subprocess.Popen([
            "xterm", "-e", "bash -c 'pkill -f ros2; exec bash'"
        ])

    @pyqtSlot()
    def home_process(self):
        print("HOME 按鈕被點擊")

    @pyqtSlot()
    def toggle_set_point_mode(self):
        """切換點位記錄模式"""
        self.recording_mode = not self.recording_mode
        if self.recording_mode:
            self.set_point_button.setStyleSheet("background-color: orange; color: white; font-size: 18px;")
            self.recorded_points = [] # 清空之前的點位
            self.directions = []# 清空之前的方向
            print("已進入點位記錄模式")
        else:
            self.set_point_button.setStyleSheet("background-color: gray; color: white; font-size: 18px;")
            self.save_points() # 儲存記錄的點位
            print("已退出點位記錄模式")

    @pyqtSlot()
    def clear_points(self):
        """清除所有記錄的點位"""
        self.recorded_points = []
        self.directions = []
        self.label.clear()
        print("所有記錄的點位已清除")

    # def save_points(self):
    #     """儲存記錄的點位到檔案"""
    #     print('self.recorded_points=', self.recorded_points)
    #     with open('saved_points.json', 'w') as file:
    #         json.dump({'points': self.recorded_points}, file)
    #         # json.dump({'points': self.recorded_points, 'directions': self.directions}, file)
    #     print("點位與方向已儲存")

    # def save_points(self):
    #     """儲存記錄的點位到檔案"""
    #     print('self.recorded_points=', self.recorded_points)
    #     with open('saved_points.json', 'w') as file:
    #         json.dump({'points': self.recorded_points}, file, indent=4)  # 使用縮排參數提高可讀性
    #     print("點位與方向已儲存")
    def save_points(self):
        """儲存記錄的點位到檔案，並限制數值小數點後 15 位"""
        print('self.recorded_points=', self.recorded_points)
        
        # 遍歷每個點，並將數值限制到小數點後 15 位
        rounded_points = []
        for point in self.recorded_points:
            rounded_point = {
                "x": round(point["x"], 15),
                "y": round(point["y"], 15),
                "z": round(point["z"], 15),
                "qx": round(point["qx"], 15),
                "qy": round(point["qy"], 15),
                "qz": round(point["qz"], 15),
                "qw": round(point["qw"], 15)
            }
            rounded_points.append(rounded_point)

        # 儲存格式化後的點位
        with open('saved_points.json', 'w') as file:
            json.dump({'points': rounded_points}, file, indent=4, separators=(',', ': '))  # 使用縮排參數提高可讀性
        
        print("點位與方向已儲存")


    def load_saved_points(self):
        """載入已儲存的點位"""
        try:
            with open('saved_points.json', 'r') as file:
                data = json.load(file)
                self.recorded_points = data['points']
                # self.directions = data['directions']
                print("已載入已儲存的點位和方向")
        except FileNotFoundError:
            self.recorded_points = []
            # self.directions = []
            print("沒有找到已儲存的點位檔案")

    def load_image_height(self, pgm_file):
        """取得圖片的實際高度（以像素為單位）"""
        image = Image.open(pgm_file)
        return image.height

    def update_map(self, index):
        """更新顯示的地圖影像與元資料"""
        if index == 0:
            # 空白選項，隱藏地圖
            self.label.clear()
            return

        selected_index = index - 1 # 因為選單中新增了一個空白選項
        self.load_map_metadata(selected_index)
        selected_pgm_file = self.pgm_files[selected_index]
        self.label.update_image(selected_pgm_file, self.load_image_height(selected_pgm_file), self.origin, self.resolution)


class MapLabel(QLabel):
    def __init__(self, parent):
        super().__init__(parent)
        self.origin = (0, 0)
        self.resolution = 1
        self.image_height = 1
        self.scale_factor = 1.0 # 初始比例設為 1.0
        self.points = [] # 儲存所有點擊的位置
        self.directions = [] # 儲存所有的方向箭頭

    def update_image(self, pgm_file, image_height, origin, resolution):
        """更新顯示的地圖影像，並自動縮放以適應視窗"""
        self.image_height = image_height
        self.origin = origin
        self.resolution = resolution
        pixmap = QPixmap(pgm_file)

        # 自動縮放以適應 QLabel 大小
        scaled_pixmap = pixmap.scaled(
        self.size(),
        Qt.KeepAspectRatio,
        Qt.SmoothTransformation
        )
        self.scale_factor = scaled_pixmap.width() / pixmap.width() # 更新縮放比例
        self.setPixmap(scaled_pixmap)
        self.update()

    def clear(self):
        """清除顯示的地圖影像"""
        # self.setPixmap(QPixmap())
        self.points.clear()
        self.directions.clear()
        self.update()


    def mousePressEvent(self, event: QMouseEvent):
        if self.window().recording_mode:
            # 呼叫自訂邏輯
            self.get_pixel_position(event)

            if len(self.points) % 2 == 0:
                # 偶數次點擊，新增點
                self.points.append(event.pos())
                world_x, world_y = self.pixel_to_world(event.pos())
                self.window().recorded_points.append({"x": world_x, "y": world_y, "z": 0.0})
                print(f"記錄世界點位: ({world_x}, {world_y})")
            else:
                # 奇數次點擊，計算方向並新增方向箭頭
                start_point = self.points[-1]
                end_point = event.pos()
                self.directions.append((start_point, end_point))
                self.points.pop() # 移除最後一個點（第二次點擊不顯示紅點）
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

    def get_pixel_position(self, event):
        px = event.pos().x()
        py = event.pos().y()
        print(f"點擊了像素位置: ({px}, {py})")

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)

        # 繪製輔助格線
        self.draw_grid(painter)

        # 繪製地圖原點
        self.draw_origin(painter)

        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("red"))

        # 繪製所有點擊位置的點
        for i, point in enumerate(self.points):
            if i % 2 == 0: # 只在第一次點擊時繪製紅色圓點
                painter.drawEllipse(QPointF(point), 5, 5) # 5x5 像素的紅色圓點

        # 繪製所有方向
        j=0
        for start_point, end_point in self.directions:
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor("red"))
            painter.drawEllipse(start_point, 5, 5) # 10x10 像素的紅色圓點
            # print("2 start_point",start_point)
            painter.setPen(QColor("blue"))
            painter.setBrush(Qt.NoBrush)
            #
            arrow_long = 50
            angle = math.atan2(end_point.y() - start_point.y(), end_point.x() - start_point.x())
            end_point = start_point + QPointF(arrow_long * math.cos(angle),arrow_long * math.sin(angle))
            #
            painter.drawLine(start_point, end_point)
            self.draw_arrow_head(painter, start_point, end_point)
            j+=1
            painter.drawText(start_point.x()+10,start_point.y()+10,str(j))#[0,1]

    def draw_grid(self, painter):
        """在地圖上繪製輔助格線"""
        grid_spacing = 50 # 格線間距為50像素
        painter.setPen(QColor(200, 200, 200)) # 灰色的格線顏色

        # 繪製垂直格線
        for x in range(0, self.width(), grid_spacing):
            painter.drawLine(x, 0, x, self.height())

        # 繪製水平格線
        for y in range(0, self.height(), grid_spacing):
            painter.drawLine(0, y, self.width(), y)

    def draw_origin(self, painter):
        """繪製地圖原點"""
        painter.setPen(QColor("red"))
        painter.setBrush(QColor("red"))

        # 計算 (0, 0) 點在地圖上的像素位置
        zero_px_x = (-self.origin[0]) / self.resolution * self.scale_factor
        zero_px_y = (self.image_height - (-self.origin[1]) / self.resolution) * self.scale_factor

        # 圓點的半徑
        radius = 5 # 設定為5像素半徑，總大小為10x10像素

        # 繪製圓點，確保圓心在原點位置
        painter.drawEllipse(int(zero_px_x) - radius, int(zero_px_y) - radius, radius * 2, radius * 2)

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
    
    # 计算四元数
    qw = math.sqrt(1 + R[0][0] + R[1][1] + R[2][2]) / 2
    
    # 添加保護機制，避免除以零
    if qw == 0:
        qw = 1e-8  # 給予一個非常小的值，避免除零錯誤
    
    qz = -((R[1][0] - R[0][1]) / (4 * qw))
    
    return 0.0, 0.0, qz, qw



if __name__ == '__main__':
    # 地圖檔案路徑
    yaml_files = [
            '/home/sr/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC.yaml',
            '/home/sr/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC1.yaml'
        ]
    pgm_files = [
            '/home/sr/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC.pgm',
            '/home/sr/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC1.pgm'
        ]

    app = QApplication(sys.argv)
    window = MapWindow(yaml_files, pgm_files)
    window.show()
    sys.exit(app.exec_())