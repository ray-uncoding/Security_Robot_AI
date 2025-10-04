
import os
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import ExecuteProcess

def generate_launch_description():
    #if use usb_cam (true or false)

    bool_usbcam = LaunchConfiguration('bool_usbcam', default='true')
    
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    usbcam_dir = get_package_share_directory('usb_cam')


    wheeltec_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_camera.launch.py')),
            condition=UnlessCondition(bool_usbcam),
    )
    wheeltec_USBcamera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(usbcam_dir, 'launch','demo.launch.py')),
            condition=IfCondition(bool_usbcam),
            launch_arguments={'video_device': '/dev/RgbCam'}.items(),
    )

    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
    )


    return LaunchDescription([
        wheeltec_camera,wheeltec_USBcamera,wheeltec_robot,

        launch_ros.actions.Node(
            condition=UnlessCondition(bool_usbcam),
            package='simple_follower_ros2', 
            executable='line_follow', 
            name='line_follow',
            parameters=[{
                'image_input':'/camera/color/image_raw',
            }]
        ),
        launch_ros.actions.Node(
            condition=IfCondition(bool_usbcam),
            package='simple_follower_ros2', 
            executable='line_follow', 
            name='line_follow',
            parameters=[{
                'image_input':'/image_raw',
            }]
        )]
    )

