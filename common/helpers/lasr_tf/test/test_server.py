#!/usr/bin/env python3
import unittest
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import TransformBroadcaster

from lasr_helper_interfaces.srv import TransformPoint
from lasr_tf.tf_server import TfServer  # update import!


class TestTfServerNoLaunch(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

        # Create nodes
        cls.tf_server_node = TfServer()
        cls.test_node = Node("tf_server_test_node")

        # Executor to spin both nodes together
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.tf_server_node)
        cls.executor.add_node(cls.test_node)

        # Client + broadcaster
        cls.cli = cls.test_node.create_client(
            TransformPoint, "/tf_server/transform_point"
        )
        cls.tf_broadcaster = TransformBroadcaster(cls.test_node)

        # Wait for service
        start = time.time()
        while not cls.cli.wait_for_service(timeout_sec=0.2):
            cls.executor.spin_once(timeout_sec=0.1)
            if time.time() - start > 5.0:
                raise RuntimeError("Service /tf_server/transform_point not available")

    @classmethod
    def tearDownClass(cls):
        cls.executor.remove_node(cls.tf_server_node)
        cls.executor.remove_node(cls.test_node)

        cls.tf_server_node.destroy_node()
        cls.test_node.destroy_node()
        rclpy.shutdown()

    def publish_tf(self):
        """Publish a test transform: map -> base_link"""
        t = TransformStamped()
        t.header.stamp = self.test_node.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = 1.0
        t.transform.translation.y = 2.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0  # identity rotation

        self.tf_broadcaster.sendTransform(t)

    def test_transform_point(self):
        # Publish TF a few times so it's stored in buffer
        for _ in range(10):
            self.publish_tf()
            self.executor.spin_once(timeout_sec=0.1)
            time.sleep(0.02)

        # Prepare request
        req = TransformPoint.Request()

        ps = PointStamped()
        ps.header.frame_id = "base_link"
        ps.header.stamp = rclpy.time.Time(
            seconds=0
        ).to_msg()  # Use Time(seconds=0) for latest transform
        ps.point.x = 1.0
        ps.point.y = 1.0
        ps.point.z = 0.0

        req.input_point_stamped = ps
        req.target_frame = "map"

        future = self.cli.call_async(req)

        # Spin until completed
        while rclpy.ok() and not future.done():
            self.executor.spin_once(timeout_sec=0.1)

        self.assertTrue(future.done())
        result = future.result()

        out = result.transformed_point_stamped.point

        # base_link(1,1,0) + translation(1,2,0) = map(2,3,0)
        self.assertAlmostEqual(out.x, 2.0, places=2)
        self.assertAlmostEqual(out.y, 3.0, places=2)
        self.assertAlmostEqual(out.z, 0.0, places=2)


if __name__ == "__main__":
    unittest.main()
