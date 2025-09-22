import sys
import os
import yaml
import subprocess
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QHBoxLayout, QPushButton, QWidget, QLineEdit, QMessageBox, QInputDialog, QScrollArea, QFrame
from PyQt5.QtGui import QPixmap, QTransform, QPainter, QColor, QPen
from PyQt5.QtCore import Qt

class MapBuilderWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.mapping_process = None
        self.save_map_process = None
        self.resolution = None  # 存儲地圖解析度
        self.current_scale = 1.0  # 初始化縮放倍率

        self.setWindowTitle('Map Builder')
        self.setFixedSize(1200, 700)  # 設定固定視窗大小

        # 主佈局
        main_layout = QVBoxLayout()

        # 狀態標籤
        self.status_label = QLabel('點擊按鈕來建立地圖', self)
        main_layout.addWidget(self.status_label)

        # 地圖名稱輸入框
        self.save_name_input = QLineEdit(self)
        self.save_name_input.setPlaceholderText("輸入地圖名稱（例如：1F）")
        main_layout.addWidget(self.save_name_input)

        # 建立按鈕區域
        button_layout = QHBoxLayout()
        self.start_button = QPushButton('啟動建圖', self)
        self.start_button.clicked.connect(self.start_mapping)
        button_layout.addWidget(self.start_button)

        self.save_button = QPushButton('保存地圖', self)
        self.save_button.clicked.connect(self.save_map)
        button_layout.addWidget(self.save_button)

        self.view_maps_button = QPushButton('查看已有地圖', self)
        self.view_maps_button.clicked.connect(self.view_existing_maps)
        button_layout.addWidget(self.view_maps_button)

        self.delete_map_button = QPushButton('刪除地圖', self)
        self.delete_map_button.clicked.connect(self.delete_map)
        button_layout.addWidget(self.delete_map_button)

        main_layout.addLayout(button_layout)

        # 地圖顯示區域
        self.scroll_area = QScrollArea(self)
        self.scroll_area.setWidgetResizable(True)

        self.map_label = QLabel(self)
        self.map_label.setAlignment(Qt.AlignCenter)
        self.scroll_area.setWidget(self.map_label)
        main_layout.addWidget(self.scroll_area)

        # 控制區域
        control_layout = QHBoxLayout()
        self.rotate_left_button = QPushButton('逆時針旋轉', self)
        self.rotate_left_button.clicked.connect(self.rotate_left)
        control_layout.addWidget(self.rotate_left_button)

        self.rotate_right_button = QPushButton('順時針旋轉', self)
        self.rotate_right_button.clicked.connect(self.rotate_right)
        control_layout.addWidget(self.rotate_right_button)

        self.zoom_in_button = QPushButton('放大', self)
        self.zoom_in_button.clicked.connect(self.zoom_in)
        control_layout.addWidget(self.zoom_in_button)

        self.zoom_out_button = QPushButton('縮小', self)
        self.zoom_out_button.clicked.connect(self.zoom_out)
        control_layout.addWidget(self.zoom_out_button)

        main_layout.addLayout(control_layout)

        # 比例尺顯示區域
        self.scale_label = QLabel('比例尺: 未知', self)
        self.scale_label.setFrameShape(QFrame.Box)
        self.scale_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(self.scale_label)

        # 設定主視窗的佈局
        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        self.current_angle = 0
        self.current_map_path = None

    def start_mapping(self):
        try:
            self.mapping_process = subprocess.Popen("bash -c 'cd ~/wheeltec_ros2 && source install/setup.bash && ros2 launch wheeltec_slam_toolbox online_async_launch.py'", shell=True, executable='/bin/bash')
            self.status_label.setText("建圖啟動中...")
        except Exception as e:
            QMessageBox.critical(self, 'Error', f"啟動建圖失敗: {e}")

    def save_map(self):
        map_name = self.save_name_input.text().strip()
        if not map_name:
            QMessageBox.warning(self, 'Warning', "請輸入地圖名稱")
            return

        save_path = f"/home/sr/wheeltec_ros2/src/wheeltec_robot_nav2/map/{map_name}"

        try:
            self.save_map_process = subprocess.Popen(f"bash -c 'cd ~/wheeltec_ros2 && source install/setup.bash && ros2 run nav2_map_server map_saver_cli -f {save_path}'", shell=True, executable='/bin/bash')
            self.status_label.setText(f"地圖保存至 {save_path}")
        except Exception as e:
            QMessageBox.critical(self, 'Error', f"保存地圖失敗: {e}")

    def view_existing_maps(self):
        map_directory = "/home/sr/wheeltec_ros2/src/wheeltec_robot_nav2/map"
        try:
            map_files = sorted([f for f in os.listdir(map_directory) if f.endswith('.pgm')])
            if map_files:
                map_file_names = [os.path.splitext(f)[0] for f in map_files]
                map_to_view, ok = QInputDialog.getItem(self, "選擇地圖", "選擇地圖：", map_file_names, 0, False)
                if ok and map_to_view:
                    self.load_map_resolution(map_directory, map_to_view + '.yaml')
                    self.show_map_image(map_directory, map_to_view + '.pgm')
            else:
                QMessageBox.information(self, "已有地圖", "目前沒有已保存的地圖檔案")
        except Exception as e:
            QMessageBox.critical(self, 'Error', f"無法讀取地圖目錄: {e}")

    def load_map_resolution(self, directory, yaml_file):
        yaml_path = os.path.join(directory, yaml_file)
        try:
            with open(yaml_path, 'r') as file:
                yaml_data = yaml.safe_load(file)
                self.resolution = yaml_data.get('resolution', None)
                if self.resolution is None:
                    QMessageBox.warning(self, 'Warning', "無法讀取地圖解析度，將使用預設比例尺")
        except Exception as e:
            QMessageBox.critical(self, 'Error', f"無法讀取 YAML 檔案: {e}")

    def show_map_image(self, directory, map_file):
        map_path = os.path.join(directory, map_file)
        self.current_map_path = map_path
        self.update_map_transform()

    def rotate_left(self):
        self.current_angle -= 10
        self.update_map_transform()

    def rotate_right(self):
        self.current_angle += 10
        self.update_map_transform()

    def zoom_in(self):
        """放大地圖時，增加縮放比例"""
        self.current_scale *= 1.1  # 每次放大 10%
        self.update_map_transform()

    def zoom_out(self):
        """縮小地圖時，減少縮放比例"""
        self.current_scale *= 0.9  # 每次縮小 10%
        self.update_map_transform()

    def update_map_transform(self):
        if self.current_map_path:
            # 加載原始圖片並應用旋轉
            original_pixmap = QPixmap(self.current_map_path)
            
            # 應用旋轉
            transform = QTransform()
            transform.rotate(self.current_angle)
            rotated_pixmap = original_pixmap.transformed(transform, Qt.SmoothTransformation)
            
            # 計算縮放比例，根據 current_scale 進行圖片縮放
            scale_factor = self.current_scale
            scaled_pixmap = rotated_pixmap.scaled(
                int(rotated_pixmap.width() * scale_factor),
                int(rotated_pixmap.height() * scale_factor),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )

            # 設置縮放後的圖片
            self.map_label.setPixmap(scaled_pixmap)

            # 繪製格線並更新比例尺
            self.draw_grid()
            self.update_scale_label()

    def draw_grid(self):
        """在地圖上繪製固定的格線"""
        if self.map_label.pixmap() is None:
            return

        pixmap = self.map_label.pixmap()
        painter = QPainter(pixmap)
        pen = QPen(QColor(200, 200, 200), 1, Qt.DotLine)
        painter.setPen(pen)

        grid_spacing = 50  # 固定的格線間距，不隨縮放變化
        for x in range(0, pixmap.width(), grid_spacing):
            painter.drawLine(x, 0, x, pixmap.height())
        for y in range(0, pixmap.height(), grid_spacing):
            painter.drawLine(0, y, pixmap.width(), y)

        painter.end()
        self.map_label.setPixmap(pixmap)

    def update_scale_label(self):
        """更新比例尺顯示，隨放大縮小變化"""
        if self.resolution is not None:
            # 根據 current_scale 計算比例尺長度
            scale_length_m = (50 * self.resolution) / self.current_scale
            rounded_length = round(scale_length_m, 2)  # 四捨五入到 0.01 米

            # 顯示比例尺
            self.scale_label.setText(f"比例尺: {rounded_length:.2f} m (50 像素)")
        else:
            self.scale_label.setText("比例尺: 未知")

    def delete_map(self):
        map_directory = "/home/sr/wheeltec_ros2/src/wheeltec_robot_nav2/map"
        try:
            map_files = [f for f in os.listdir(map_directory) if f.endswith('.yaml') or f.endswith('.pgm')]
            if not map_files:
                QMessageBox.information(self, "刪除地圖", "目前沒有可刪除的地圖檔案")
                return

            map_file_names = [os.path.splitext(f)[0] for f in map_files]
            map_to_delete, ok = QInputDialog.getItem(self, "選擇要刪除的檔案", "選擇檔案：", sorted(set(map_file_names)), 0, False)
            if ok and map_to_delete:
                yaml_file_path = os.path.join(map_directory, f"{map_to_delete}.yaml")
                pgm_file_path = os.path.join(map_directory, f"{map_to_delete}.pgm")
                if os.path.exists(yaml_file_path):
                    os.remove(yaml_file_path)
                if os.path.exists(pgm_file_path):
                    os.remove(pgm_file_path)
                QMessageBox.information(self, "刪除地圖", f"已刪除地圖：{map_to_delete}")
            else:
                QMessageBox.information(self, "刪除地圖", "未選擇任何檔案")
        except Exception as e:
            QMessageBox.critical(self, 'Error', f"無法刪除地圖檔案: {e}")

    def closeEvent(self, event):
        reply = QMessageBox.question(self, '退出', "確定要退出並停止建圖進程嗎?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.stop_mapping()
            event.accept()
        else:
            event.ignore()

    def stop_mapping(self):
        if self.mapping_process:
            self.mapping_process.terminate()
            self.mapping_process.wait()
            self.mapping_process = None
        if self.save_map_process:
            self.save_map_process.terminate()
            self.save_map_process.wait()
            self.save_map_process = None
        self.status_label.setText("建圖進程已停止")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MapBuilderWindow()
    window.show()
    sys.exit(app.exec_())
