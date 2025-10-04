
import time
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    waypoints = [
        {
            "x": -0.5712810843828311,
            "y": -3.5432846949234365,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": -0.15394943104317801,
            "qw": 0.9880787279774228
        },
        {
            "x": 1.2077418728120726,
            "y": -3.6131249397343734,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.6606588991112498,
            "qw": 0.7506862320737683
        },
        {
            "x": 5.4109198438036215,
            "y": 0.4012809838948602,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.6977885684820497,
            "qw": 0.7163037859007669
        },
        {
            "x": 5.578655403644704,
            "y": 2.782073451495792,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.9929940622784668,
            "qw": 0.11816425973918068
        },

        {
            "x": 3.1718655996877505,
            "y": 2.8979294892565557,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": -0.796202960047208,
            "qw": 0.6050296244086434
        },

        {
            "x": 2.959936223064883,
            "y": 0.49194696489205514,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": -0.11142263058050911,
            "qw": 0.9937731116278602
        },

        {
            "x": 7.052615920911385,
            "y": 13.164571988664958,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": -0.031096625099156693,
            "qw": 0.9995163830110252
        },

        {
            "x": 8.90201896902107,
            "y": 13.274004911236965,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": -0.7453281994051888,
            "qw": 0.6666977389877808
        },

        {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": -0.027882165379993294,
            "qw": 0.9996112168506928

        },
    ]

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
                
            time.sleep(3)

    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
