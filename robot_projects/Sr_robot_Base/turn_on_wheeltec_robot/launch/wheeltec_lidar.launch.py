import os
from pathlib import Path
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                             IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
     # 取得 rplidar_ros 套件的共用目錄路徑
     sllidar_dir = get_package_share_directory('rplidar_ros')
     sllidar_launch_dir = os.path.join(sllidar_dir, 'launch')

     # 定義 sllidar_s2_launch.py 的啟動檔描述
     sllidar_a2m12 = IncludeLaunchDescription(
         PythonLaunchDescriptionSource(os.path.join(sllidar_launch_dir, 'rplidar_a2m12_launch.py')),
     )
                  
     # 建立啟動描述對象
     ld = LaunchDescription()
    
     # 新增 sllidar_s2 啟動檔案描述到 LaunchDescription 物件中
     ld.add_action(sllidar_a2m12)

     # 傳回啟動描述對象
     return ld