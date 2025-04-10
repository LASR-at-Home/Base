import rclpy
from rclpy.node import Node


class GreetNode(Node):
    def __init__(self):
        super().__init__("greet_node")
        self.get_logger().info("Hello from Greet Node!")


def main(args=None):
    rclpy.init(args=args)
    node = GreetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
