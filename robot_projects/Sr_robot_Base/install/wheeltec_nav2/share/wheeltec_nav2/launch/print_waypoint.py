import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class AmclSubscriber(Node):

    def __init__(self):
        super().__init__('robotpose_subscriber')
        self.get_logger().info('Starting AMCL Subscriber node...')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/robotpose',  # /topic名稱
            self.amcl_callback,
            10)
        self.subscription 

    def amcl_callback(self, msg):
        self.get_logger().info('Received message from /amcl_pose topic')
        x = msg.x
        y = msg.y
        z = msg.z
        self.get_logger().info(f'Position: x={x}, y={y}, z={z}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    robotpose_subscriber = AmclSubscriber()
    rclpy.spin(robotpose_subscriber)
    robotpose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
