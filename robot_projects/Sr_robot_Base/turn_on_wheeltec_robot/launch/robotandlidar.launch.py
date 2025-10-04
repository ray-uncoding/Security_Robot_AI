
# 匯入必要模組
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

#def launch(launch_descriptor, argv):
def generate_launch_description():
        # 取得 rplidar_ros 套件的 launch 目錄
        rplidar_dir = get_package_share_directory('rplidar_ros')
        rplidar_launch_dir = os.path.join(rplidar_dir, 'launch')

        # 取得 turn_on_wheeltec_robot 套件的 launch 目錄
        bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
        launch_dir = os.path.join(bringup_dir, 'launch')

        # 啟動 wheeltec_robot 的 launch 檔案
        wheeltec_robot = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
        )

        # ⚠️ 這裡原本應該啟動 rplidar_ros 的 launch 檔案
        # 但目前是重複啟動 turn_on_wheeltec_robot.launch.py
        # 正確做法應該是：
        # rplidar_ros = IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(rplidar_launch_dir, 'rplidar.launch.py')),
        # )
        # 目前這樣會導致雷達和底盤都啟動同一份設定，可能造成資源衝突或初始化錯誤
        rplidar_ros = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
        )

        # 回傳 LaunchDescription，包含 wheeltec_robot 和 rplidar_ros
        # ⚠️ 目前兩者都啟動同一份 launch 檔案，建議後續修正
        return LaunchDescription([
                wheeltec_robot,
                rplidar_ros,
        ])

