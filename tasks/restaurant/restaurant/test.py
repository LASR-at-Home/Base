import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class AmclPoseSubscriber(Node):

    def __init__(self):
        super().__init__('amcl_pose_subscriber')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.listener_callback,
            10)
        self.get_logger().info(f'Current Position: x')


    def listener_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.get_logger().info(f'Current Position: x={position.x:.2f}, y={position.y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    amcl_pose_subscriber = AmclPoseSubscriber()
    rclpy.spin(amcl_pose_subscriber)
    amcl_pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
