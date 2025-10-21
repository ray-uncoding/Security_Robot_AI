# Sr_robot_Base
- 系統版本：Ubuntu 22.04 Ros2 humble 
- Lidar型號：Rplidar a2m12

## 安裝依賴
sudo apt install ros-humble-test-msgs* -y
sudo apt install ros-humble-behaviortree-cpp-v3* -y
sudo apt install ros-humble-ompl -y
sudo apt install ros-humble-async-web-server-cpp* -y
sudo apt install ros-humble-filters -y
sudo apt install ros-humble-diagnostic-updater -y

## 一定要做
## colconbuld 執行兩次是因為確保所有安裝包被完整打包
colcon build --packages-select wheeltec_rrt_msg
colcon build --packages-select wheeltec_robot_rrt
colcon build
colcon build

## 有問題再說
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:~/Sr_robot_Base/install
colcon build --packages-select nav2_behavior_tree
colcon build --packages-select nav2_costmap_2d
colcon build --packages-select nav2_rviz_plugins
colcon build --packages-select nav2_mppi_controller
colcon build --packages-select serial_ros2

## 
colcon build --packages-select lslidar_driver
colcon build --packages-select simple_follower_ros2
colcon build --packages-select simple_slam_gmapping

colcon build --packages-select costmap_queue

# colcon build --packages-select astra_camera