import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class WaitForMessageNode(Node):
    def __init__(self, name="skills_access_node"):
        super().__init__(name)

    def wait_for_message(self, topic, msg_type, timeout=5.0):
        """
        ROS2 does not provide wait_for_message
        Waits for a message with a Future object (More efficient).
        """
        future = Future()
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        def callback(msg):
            if not future.done():
                future.set_result(msg)

        self.subscriber = self.create_subscription(
            msg_type, topic, callback, qos_profile
        )

        start_time = self.get_clock().now().nanoseconds / 1e9

        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed_time = (self.get_clock().now().nanoseconds / 1e9) - start_time
            if elapsed_time > timeout:
                return None

        return future.result()


class AccessNode(Node):
    """
    Class to  create and access the node to avoid duplications
    """

    _node = None  # Static variable to hold the node instance

    @staticmethod
    def get_node():
        """Returns the singleton ROS 2 node instance, creating it if necessary."""
        if AccessNode._node is None:
            AccessNode._node = WaitForMessageNode(name="skills_access_node")
        return AccessNode._node

    @staticmethod
    def shutdown():
        """Shuts down the singleton node properly."""
        if AccessNode._node is not None:
            AccessNode._node.destroy_node()
            AccessNode._node = None
            AccessNode.shutdown()
