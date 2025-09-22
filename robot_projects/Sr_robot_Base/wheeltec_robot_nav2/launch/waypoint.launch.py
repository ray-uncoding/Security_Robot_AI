import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult  # 假設這些類別來自某個導航套件

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.navigator = BasicNavigator(self)
        self.waypoints = [
            {"x": 0.8468844890594482, "y": -0.0845317393541336, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.5323170593657491, "qw": 0.8465450657278687},
            {"x": 0.963241457939148, "y": 0.5066562294960022, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.7499148066360094, "qw": 0.6615344154222641},
            {"x": 0.6496784687042236, "y": 0.9676530957221985, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.9714516320878575, "qw": 0.2372377004479645},
            {"x": 0.17888396978378296, "y": 1.0539872646331787, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": -0.9463608910839065, "qw": 0.3231115346546367},
            {"x": -0.13098974525928497, "y": 0.5612476468086243, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
        ]

    def navigate_waypoints(self):
        goal_poses = []
        for wp in self.waypoints:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = wp["x"]
            goal_pose.pose.position.y = wp["y"]
            goal_pose.pose.position.z = wp["z"]
            goal_pose.pose.orientation.x = wp["qx"]
            goal_pose.pose.orientation.y = wp["qy"]
            goal_pose.pose.orientation.z = wp["qz"]
            goal_pose.pose.orientation.w = wp["qw"]
            goal_poses.append(goal_pose)

        for goal_pose in goal_poses:
            self.navigator.goThroughPoses([goal_pose])

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    self.get_logger().info('Distance remaining: {:.2f} meters'.format(feedback.distance_remaining))

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                self.get_logger().info('Goal was canceled!')
            elif result == TaskResult.FAILED:
                self.get_logger().info('Goal failed!')

def main(args=None):
    rclpy.init(args=args)
    waypoint_navigator = WaypointNavigator()
    rclpy.spin(waypoint_navigator)
    waypoint_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# ros2 launch wheeltec_nav2 waypoint.launch.py 
# [INFO] [launch]: All log files can be found below /home/sr/.ros/log/2024-08-11-18-13-22-322026-sr-253285
# [INFO] [launch]: Default logging verbosity is set to INFO
# [ERROR] [launch]: Caught exception in launch (see debug for traceback): Caught multiple exceptions when trying to load file of format [py]:
#  - InvalidPythonLaunchFileError: launch file at '/home/sr/wheeltec_ros2/install/wheeltec_nav2/share/wheeltec_nav2/launch/waypoint.launch.py' does not contain the required function 'generate_launch_description()'
#  - InvalidFrontendLaunchFileError: The launch file may have a syntax error, or its format is unknown
