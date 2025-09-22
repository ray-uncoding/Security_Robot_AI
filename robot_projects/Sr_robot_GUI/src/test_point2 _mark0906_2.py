import sys
import subprocess
import math
import yaml
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget, QComboBox, QToolBar, QPushButton, QHBoxLayout
from PyQt5.QtGui import QPixmap, QPainter, QColor, QMouseEvent, QPolygonF
from PyQt5.QtCore import Qt, QPointF, pyqtSlot, QSize, QRectF

class SimpleMapWindow(QMainWindow):
    def __init__(self, yaml_files, pgm_files):
        super().__init__()
        self.yaml_files = yaml_files
        self.pgm_files = pgm_files
        self.current_map_index = 0

        self.initUI()
        

    def initUI(self):
        """初始化用户界面"""
        self.setWindowTitle('Map with Control Buttons, Image Selector, and Grid')
        self.setFixedSize(1300, 1000)  # 设置固定窗口大小

        # 创建工具栏和下拉式选择菜单
        toolbar = QToolBar(self)
        self.addToolBar(toolbar)

        map_selector = QComboBox(self)
        for i, pgm_file in enumerate(self.pgm_files):
            map_selector.addItem(f'地图 {i+1}', i)

        map_selector.currentIndexChanged.connect(self.switch_map)
        toolbar.addWidget(map_selector)

        # 创建按钮并设置其样式
        self.start_button = QPushButton("START\n启用")
        self.stop_button = QPushButton("STOP\n停止")
        self.home_button = QPushButton("HOME\n回原点")
        self.set_point_button = QPushButton("SET POINT\n设置点位")
        self.clear_button = QPushButton("CLEAR\n清除点位")

        # 绑定按钮事件
        self.start_button.clicked.connect(self.start_process)
        self.stop_button.clicked.connect(self.stop_process)
        self.home_button.clicked.connect(self.home_process)
        self.set_point_button.clicked.connect(self.toggle_set_point_mode)
        self.clear_button.clicked.connect(self.clear_points)

        # 设置按钮大小和颜色
        button_size = QSize(140, 160)
        self.start_button.setFixedSize(button_size)
        self.start_button.setStyleSheet("background-color: green; color: white; font-size: 18px;")
        self.stop_button.setFixedSize(button_size)
        self.stop_button.setStyleSheet("background-color: red; color: white; font-size: 18px;")
        self.home_button.setFixedSize(button_size)
        self.home_button.setStyleSheet("background-color: blue; color: white; font-size: 18px;")
        self.set_point_button.setFixedSize(button_size)
        self.set_point_button.setStyleSheet("background-color: gray; color: white; font-size: 18px;")
        self.clear_button.setFixedSize(button_size)
        self.clear_button.setStyleSheet("background-color: orange; color: white; font-size: 18px;")

        # 创建按钮布局
        button_layout = QVBoxLayout()
        button_layout.setContentsMargins(30, 30, 30, 30)
        button_layout.setSpacing(30)  # 增加按钮之间的间距
        button_layout.addWidget(self.start_button)
        button_layout.addWidget(self.stop_button)
        button_layout.addWidget(self.home_button)
        button_layout.addWidget(self.set_point_button)
        button_layout.addWidget(self.clear_button)
        button_layout.addStretch()  # 添加伸缩空间，确保按钮在顶部

        # 创建地图标签并加载地图图像
        self.label = MapLabel(self.load_map_metadata(), self.pgm_files[self.current_map_index], self)
        self.origin=self.label.map_metadata['origin']#
        print('self.origin=',self.origin)
        # 创建主布局，将按钮和地图布局结合在一起
        main_layout = QHBoxLayout()
        main_layout.addLayout(button_layout)  # 左侧按钮布局
        main_layout.addWidget(self.label)     # 右侧地图布局

        # 创建主窗口部件并设置布局
        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

    def load_map_metadata(self):
        """加载当前地图的元数据信息"""
        with open(self.yaml_files[self.current_map_index], 'r') as file:
            map_metadata = yaml.safe_load(file)
            
        return map_metadata

    def switch_map(self, index):
        """切换地图"""
        self.current_map_index = index
        map_metadata = self.load_map_metadata()
        self.label.update_map(map_metadata, self.pgm_files[self.current_map_index])

    @pyqtSlot()
    def start_process(self):
        print("START 按钮被点击")
        subprocess.Popen([
            "xterm", "-e", "bash -c 'cd ~/wheeltec_ros2 && source install/setup.bash && ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py; exec bash'"
        ])

    @pyqtSlot()
    def stop_process(self):
        print("STOP 按钮被点击")
        subprocess.Popen([
            "xterm", "-e", "bash -c 'pkill -f ros2; exec bash'"
        ])

    @pyqtSlot()
    def home_process(self):
        print("HOME 按钮被点击")
        # 在这里实现回到原点的逻辑

    @pyqtSlot()
    def toggle_set_point_mode(self):
        print("SET POINT 按钮被点击")
        # 在这里实现切换设置点位模式的逻辑

    @pyqtSlot()
    def clear_points(self):
        print("CLEAR 按钮被点击")
        self.label.clear_points()

class MapLabel(QLabel):
    def __init__(self, map_metadata, map_image_path, parent):
        super().__init__(parent)
        self.map_metadata = map_metadata
        self.map_image_path = map_image_path
        self.scale_factor = 1.0
        self.waypoints=[]##
        self.resolution=self.map_metadata['resolution']
        self.initMap(map_image_path)

    def initMap(self, map_image_path):
        """初始化地图和绘图相关的属性"""
        self.pixmap = QPixmap(map_image_path)
        self.points = []  # 存储所有点击的位置
        self.directions = []  # 存储所有的方向箭头

        # 自动缩放地图以适应窗口大小
        self.update_image()

    def update_map(self, map_metadata, map_image_path):
        """更新地图"""
        self.map_metadata = map_metadata
        self.map_image_path = map_image_path
        self.update_image()

    def update_image(self):
        """更新显示的地图图像，并自动缩放以适应窗口"""
        pixmap = QPixmap(self.map_image_path)

        # 自动缩放以适应 QLabel 大小
        scaled_pixmap = pixmap.scaled(
            self.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
        self.scale_factor = scaled_pixmap.width() / pixmap.width()  # 更新缩放比例
        self.setPixmap(scaled_pixmap)
        self.update()

    def clear_points(self):
        """清除所有点位"""
        self.points.clear()
        self.directions.clear()
        self.update()

    def mousePressEvent(self, event: QMouseEvent):
        """处理鼠标点击事件"""
        self.handleMouseClick(event.pos())
        self.update()

    def handleMouseClick(self, position):
        """处理点击的具体逻辑"""
        if len(self.points) % 2 == 0:
            self.points.append(position)
        else:
            start_point = self.points[-1]
            end_point = position
            self.directions.append((start_point, end_point))
            self.points.pop()

#    def paintEvent(self, event):
#        super().paintEvent(event)
#        painter = QPainter(self)
#        self.drawGrid(painter)
#        self.drawPoints(painter)
#        self.drawDirections(painter)
#        self.drawOrigin(painter)  # 绘制原点
    def mousePressEvent(self, event: QMouseEvent):
        # 获取鼠标点击的位置
        #if len(self.waypoints) % 2 == 0:
            # 偶数次点击，添加新点
        #realX=event.x()*self.resolution/self.scale_factor
        #realY=event.y()*self.resolution/self.scale_factor
        self.waypoints.append([event.x(),event.y()])
        #else:
            # 奇数次点击，计算方向并添加方向箭头
            #start_point = self.waypoints[-1]
            #end_point = event.pos()
            #self.directions.append((start_point, end_point))  # 存储方向
            #self.waypoints.pop()  # 移除最后一个点（第二次点击不显示红点）
        # print('test',self.waypoints)
        self.update()
    def paintEvent(self, event):
            super().paintEvent(event)
            painter = QPainter(self)
            self.drawGrid(painter)
            painter.setPen(QColor("red"))
    
            # 计算 (0, 0) 点在地图上的像素位置

            zero_px_x = (-self.map_metadata['origin'][0]) / self.map_metadata['resolution'] * self.scale_factor
            zero_px_y = (self.pixmap.height() - (-self.map_metadata['origin'][1]) / self.map_metadata['resolution']) * self.scale_factor
    
            # 绘制红点
            # print('0:',zero_px_x,zero_px_y)
            painter.drawEllipse(int(zero_px_x), int(zero_px_y), 5, 5)  # 10x10 像素的圆点
    
            # 绘制每个点位及其方向
            i=0
            begin_px_x=zero_px_x
            begin_px_y=zero_px_y
            for waypoint in self.waypoints:
                # 将世界坐标转换为像素坐标
                #px_x = (waypoint[0] - self.map_metadata['origin'][0]) / self.resolution * self.scale_factor
                #px_y = -(self.pixmap.height() - (waypoint[1] - self.map_metadata['origin'][1]) / self.resolution) * self.scale_factor
                px_x=waypoint[0]
                px_y=waypoint[1]
                # 绘制点位
                painter.setBrush(QColor("yellow"))
                i+=1
                if i%2==0:#畫起點
                    painter.drawEllipse(int(px_x), int(px_y), 10, 10)
                    begin_px_x=px_x
                    begin_px_y=px_y
                    painter.drawText(int(px_x+10), int(px_y+10),str(int(i/2)))#顯示第幾站
                    x = waypoint[0] * self.resolution / self.scale_factor + self.map_metadata['origin'][0]
                    y = (self.pixmap.height()-waypoint[1]) * self.resolution / self.scale_factor - self.map_metadata['origin'][1]
                    print("像素座標",int(i/2),":",waypoint[0],waypoint[1])
                    print("世界座標",int(i/2),":",x,y)
                else: 
                    start_point = QPointF(int(begin_px_x), int(begin_px_y) )
                    end_point = QPointF(int(px_x), int(px_y) )
                    # self.drawArrowHead(painter, start_point, end_point)
                    self.drawDirections(painter,start_point, end_point)

    
            painter.end()
    def drawGrid(self, painter):
        """在地图上绘制辅助格线"""
        grid_spacing = 50 * self.scale_factor  # 根据缩放比例调整网格大小
        painter.setPen(QColor(200, 200, 200))  # 灰色的格线颜色

        # 绘制垂直格线
        for x in range(0, self.width(), int(grid_spacing)):
            painter.drawLine(x, 0, x, self.height())

        # 绘制水平格线
        for y in range(0, self.height(), int(grid_spacing)):
            painter.drawLine(0, y, self.width(), y)

    def drawPoints(self, painter):
        """绘制点击的位置"""
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("red"))

        for i, point in enumerate(self.points):
            if i % 2 == 0:
                painter.drawEllipse(QPointF(point), 10 * self.scale_factor, 10 * self.scale_factor)  # 根据缩放比例调整点的大小

    # def drawDirections(self, painter):
    #     """绘制方向箭头"""
    #     painter.setPen(QColor("blue"))
    #     painter.setBrush(Qt.NoBrush)
    #     for start_point, end_point in self.directions:
    #         painter.drawLine(start_point, end_point)
    #         self.drawArrowHead(painter, start_point, end_point)
    def drawDirections(self, painter,start_point, end_point):
        """绘制方向箭头"""
        painter.setPen(QColor("blue"))
        painter.setBrush(Qt.NoBrush)
        painter.drawLine(start_point, end_point)
        self.drawArrowHead(painter, start_point, end_point)
    def drawOrigin(self, painter):
        """绘制地图原点"""
        painter.setPen(QColor("red"))
        painter.setBrush(QColor("red"))

        # 计算 (0, 0) 点在地图上的像素位置
        zero_px_x = (-self.map_metadata['origin'][0]) / self.map_metadata['resolution'] * self.scale_factor
        zero_px_y = (self.pixmap.height() - (-self.map_metadata['origin'][1]) / self.map_metadata['resolution']) * self.scale_factor

        # 圆点的半径
        radius = 5  # 设置为5像素半径，总大小为10x10像素

        # 绘制圆点，确保圆心在原点位置
        painter.drawEllipse(int(zero_px_x) - radius, int(zero_px_y) - radius, radius * 2, radius * 2)

    def drawArrowHead(self, painter, start_point, end_point):
        """绘制箭头"""
        arrow_size = 10 * self.scale_factor  # 调整箭头大小以适应缩放
        angle = math.atan2(start_point.y() - end_point.y(), start_point.x() - end_point.x())

        arrow_p1 = end_point + QPointF(arrow_size * math.cos(angle + math.pi / 6),
                                       arrow_size * math.sin(angle + math.pi / 6))
        arrow_p2 = end_point + QPointF(arrow_size * math.cos(angle - math.pi / 6),
                                       arrow_size * math.sin(angle - math.pi / 6))

        arrow_head = QPolygonF([end_point, arrow_p1, arrow_p2])
        painter.drawPolygon(arrow_head)

def main():
    """主函数，启动应用程序"""
    # 地图文件路径
    yaml_files = [
        '/home/sr/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC.yaml',
        '/home/sr/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC1.yaml'
    ]
    pgm_files = [
        '/home/sr/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC.pgm',
        '/home/sr/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC1.pgm'
    ]

    app = QApplication(sys.argv)
    window = SimpleMapWindow(yaml_files, pgm_files)
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
