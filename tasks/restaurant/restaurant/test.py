import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped

class AmclPoseSubscriber(Node):

    def __init__(self):
        super().__init__('amcl_pose_subscriber')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.listener_callback,
            qos)
        self.get_logger().info('AmclPoseSubscriber started, subscription created for /amcl_pose')
        domain = os.environ.get('ROS_DOMAIN_ID', 'not set')
        self.get_logger().info(f'ROS_DOMAIN_ID={domain}')
        try:
            pubs = self.get_publishers_info_by_topic('/amcl_pose')
            self.get_logger().info(f'Publishers for /amcl_pose: {pubs}')
        except Exception as e:
            self.get_logger().warn(f'Could not get publishers info: {e}')


    def listener_callback(self, msg):
        self.get_logger().debug('listener_callback invoked')
        try:
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            self.get_logger().info(f'Current Position: x={position.x:.2f}, y={position.y:.2f}')
        except Exception as e:
            self.get_logger().error(f'Exception in listener_callback: {e}')

def main(args=None):
    rclpy.init(args=args)
    amcl_pose_subscriber = AmclPoseSubscriber()
    rclpy.spin(amcl_pose_subscriber)
    amcl_pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
