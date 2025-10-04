import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def navigate_to_home():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0.0  # 設定原點座標
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.w = 1.0  # 沒有旋轉

    print("導航回到原點")
    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f'回原點中的剩餘距離: {feedback.distance_remaining:.2f} meters')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("成功回到原點")
    else:
        print("回原點失敗")

    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    navigate_to_home()
