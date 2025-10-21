import time
import json
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# # 讀取儲存的點位
# def load_waypoints_from_json(file_path):
#     with open(file_path, 'r') as file:
#         data = json.load(file)
#     return data["points"]

# 讀取儲存的點位
def load_waypoints_from_json(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    
    waypoints = data["points"]

    # 列印讀取到的點位，確認是否成功讀取
    print("讀取到的點位：")
    for i, waypoint in enumerate(waypoints):
        print(f"點 {i+1}: {waypoint}")
    
    return waypoints

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    # 讀取已儲存的點位
    # waypoints = load_waypoints_from_json('/home/sr/gui_ws/saved_points.json')
    waypoints = load_waypoints_from_json('saved_points.json') # 於當前目錄下尋找該檔案

    goal_poses = []
    for wp in waypoints:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = wp["x"]
        goal_pose.pose.position.y = wp["y"]
        goal_pose.pose.position.z = wp["z"]
        goal_pose.pose.orientation.x = wp["qx"]
        goal_pose.pose.orientation.y = wp["qy"]
        goal_pose.pose.orientation.z = wp["qz"]
        goal_pose.pose.orientation.w = wp["qw"]
        goal_poses.append(goal_pose)

    while rclpy.ok():
        for goal_pose in goal_poses:
            navigator.goThroughPoses([goal_pose])

            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                if feedback:
                    print('Distance remaining: {:.2f} meters'.format(feedback.distance_remaining))

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')
                
            time.sleep(30)

    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
