import sys
import os
import math
import yaml
import subprocess
from PyQt5.QtWidgets import (
    QApplication, QLabel, QMainWindow, QVBoxLayout, QHBoxLayout,
    QPushButton, QWidget, QLineEdit, QMessageBox, QInputDialog,
    QScrollArea, QFrame, QFileDialog
)
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QPixmap, QPainter, QColor, QPen


class MapBuilderWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.mapping_process = None
        self.save_map_process = None
        self.resolution = None  # 地圖解析度（每像素多少米）
        self.origin = (0, 0)  # 地圖原點位置
        self.current_scale = 1.0  # 初始化縮放比例
        self.drawing_mode = "brush"  # 默認為畫筆模式
        self.brush_size = 10  # 畫筆大小

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

        # 畫筆與橡皮擦工具
        tool_layout = QHBoxLayout()
        self.brush_button = QPushButton('畫筆', self)
        self.brush_button.clicked.connect(self.set_brush_mode)
        tool_layout.addWidget(self.brush_button)

        self.eraser_button = QPushButton('橡皮擦', self)
        self.eraser_button.clicked.connect(self.set_eraser_mode)
        tool_layout.addWidget(self.eraser_button)

        self.save_image_button = QPushButton('保存修改', self)
        self.save_image_button.clicked.connect(self.save_edited_map)
        tool_layout.addWidget(self.save_image_button)

        main_layout.addLayout(tool_layout)

        # 控制區域
        control_layout = QHBoxLayout()
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

        self.current_map_path = None
        self.edited_pixmap = None  # 用於存儲可編輯的地圖

    def start_mapping(self):
        try:
            self.mapping_process = subprocess.Popen(
                "bash -c 'cd ~/wheeltec_ros2 && source install/setup.bash && ros2 launch wheeltec_slam_toolbox online_async_launch.py'",
                shell=True, executable='/bin/bash'
            )
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
            self.save_map_process = subprocess.Popen(
                f"bash -c 'cd ~/wheeltec_ros2 && source install/setup.bash && ros2 run nav2_map_server map_saver_cli -f {save_path}'",
                shell=True, executable='/bin/bash'
            )
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
                self.origin = tuple(yaml_data.get('origin', (0, 0)))
                if self.resolution is None:
                    QMessageBox.warning(self, 'Warning', "無法讀取地圖解析度，將使用預設比例尺")
        except Exception as e:
            QMessageBox.critical(self, 'Error', f"無法讀取 YAML 檔案: {e}")

    def show_map_image(self, directory, map_file):
        map_path = os.path.join(directory, map_file)
        self.current_map_path = map_path
        self.update_map_transform()

    def zoom_in(self):
        self.current_scale += 0.1
        self.update_map_transform()

    def zoom_out(self):
        if self.current_scale > 0.1:
            self.current_scale -= 0.1
            self.update_map_transform()

    def update_map_transform(self):
        if self.current_map_path:
            original_pixmap = QPixmap(self.current_map_path)
            scaled_pixmap = original_pixmap.scaled(
                int(original_pixmap.width() * self.current_scale),
                int(original_pixmap.height() * self.current_scale),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            self.edited_pixmap = scaled_pixmap.copy()
            self.map_label.setPixmap(self.edited_pixmap)
            self.update_scale_label()

    def set_brush_mode(self):
        self.drawing_mode = "brush"

    def set_eraser_mode(self):
        self.drawing_mode = "eraser"

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self.edited_pixmap is not None:
            self.modify_map(event)

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.LeftButton and self.edited_pixmap is not None:
            self.modify_map(event)

    def modify_map(self, event):
        x = event.pos().x() - self.map_label.x()
        y = event.pos().y() - self.map_label.y()
        painter = QPainter(self.edited_pixmap)
        pen = QPen()
        if self.drawing_mode == "brush":
            pen.setColor(Qt.black)
        elif self.drawing_mode == "eraser":
            pen.setColor(Qt.white)
        pen.setWidth(self.brush_size)
        painter.setPen(pen)
        painter.drawPoint(x, y)
        painter.end()
        self.map_label.setPixmap(self.edited_pixmap)

    def save_edited_map(self):
        if self.edited_pixmap:
            save_path = QFileDialog.getSaveFileName(self, "保存地圖", "", "Images (*.png *.jpg *.bmp)")[0]
            if save_path:
                self.edited_pixmap.save(save_path)
                QMessageBox.information(self, "保存成功", f"地圖已保存至: {save_path}")

    def update_scale_label(self):
        if self.resolution:
            scale_length = (50 * self.resolution) / self.current_scale
            self.scale_label.setText(f"比例尺: {round(scale_length, 2):.2f} m (50 像素)")
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


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MapBuilderWindow()
    window.show()
    sys.exit(app.exec_())
