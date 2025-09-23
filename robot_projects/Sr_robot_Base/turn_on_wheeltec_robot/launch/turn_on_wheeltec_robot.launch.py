# ====================================================
# ========== Step 1 基本匯入與初始化 =============
# ====================================================

import os
from pathlib import Path                                                        # 用於處理檔案路徑
import launch                                                                   # ROS2 的啟動系統
from launch.actions import SetEnvironmentVariable                               # 用於設定環境變數
from ament_index_python.packages import get_package_share_directory             # 用於取得 ROS2 套件的分享目錄
from launch import LaunchDescription                                            # 用於描述啟動設定
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)   # 用於啟動其他 launch 檔案
from launch.launch_description_sources import PythonLaunchDescriptionSource     # 用於包含其他 Python launch 檔案
from launch.substitutions import LaunchConfiguration, PythonExpression          # 用於處理啟動參數
from launch_ros.actions import PushRosNamespace                                 # 用於設定 ROS 命名空間
import launch_ros.actions                                                       # ROS2 的啟動系統
from launch.conditions import IfCondition                                       # 用於條件啟動
from launch.conditions import UnlessCondition                                   # 用於條件啟動

# ====================================================
# ========== Step 2 設定 Launch 參數 =============
# ====================================================

# 2.1 生成 Launch 描述
def generate_launch_description():
    
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')         # 取得 bringup 套件的分享目錄
    launch_dir = os.path.join(bringup_dir, 'launch')                            # 組合出 launch 目錄的完整路徑
        
    ekf_config = Path(get_package_share_directory('turn_on_wheeltec_robot'), 'config', 'ekf.yaml')                      # EKF 設定檔路徑
    ekf_carto_config = Path(get_package_share_directory('turn_on_wheeltec_robot'), 'config', 'ekf_carto.yaml')          # EKF Carto SLAM 設定檔路徑

    imu_config = Path(get_package_share_directory('turn_on_wheeltec_robot'), 'config', 'imu.yaml')                      # IMU 設定檔路徑

    
    carto_slam = LaunchConfiguration('carto_slam', default='false')             # 是否啟用 Carto SLAM 的參數
    
    carto_slam_dec = DeclareLaunchArgument('carto_slam',default_value='false')  # 宣告啟動參數
            
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'base_serial.launch.py')),
            launch_arguments={'akmcar': 'false'}.items(),
    )
    #choose your car,the default car is mini_mec 
    choose_car = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_mode_description.launch.py')),
    )
        
    robot_ekf = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_ekf.launch.py')),
            launch_arguments={'carto_slam':carto_slam}.items(),            
    )

                                                            
    base_to_link = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_link',
            arguments=['0', '0', '0','0', '0','0','base_footprint','base_link'],
    )
    base_to_gyro = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_gyro',
            arguments=['0', '0', '0','0', '0','0','base_footprint','gyro_link'],
    )
    
    imu_filter_node =  launch_ros.actions.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_config]
    )
    
                           
    joint_state_publisher_node = launch_ros.actions.Node(
            package='joint_state_publisher', 
            executable='joint_state_publisher', 
            name='joint_state_publisher',
    )

    ld = LaunchDescription()

    ld.add_action(choose_car)
    ld.add_action(carto_slam_dec)
    ld.add_action(wheeltec_robot)
    ld.add_action(base_to_link)
    ld.add_action(base_to_gyro)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(imu_filter_node)    
    ld.add_action(robot_ekf)

    return ld

