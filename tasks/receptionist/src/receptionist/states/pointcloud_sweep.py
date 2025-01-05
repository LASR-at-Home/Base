#!/usr/bin/env python3
from typing import List, Tuple, Optional
import smach
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
import tf2_ros as tf

from tf_pcl import pcl_transform
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import PointCloud2

from lasr_skills import LookToPoint

# TODO merge tf_pcl to this branch & test this SM

# global tf buffer
tf_buffer = tf.Buffer(cache_time=Duration(seconds=10))

def start_tf_buffer() -> None:
    tf.TransformListener(tf_buffer)


class PointCloudSweep(smach.StateMachine):
    def __init__(self, sweep_points: List[Point]):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["transformed_pointclouds"],
        )
        self.sweep_points = sweep_points

        start_tf_buffer()
        with self:
            self.userdata.transformed_pointclouds = []
            for index, point in enumerate(sweep_points):
                smach.StateMachine.add(
                    f"GetPointStamped_{index}",
                    self.GetPointStamped(point=point),
                    transitions={
                        "succeeded": f"LookToPoint_{index}",
                        "failed": "failed",
                    },
                    remapping={
                        "pointstamped": f"pointstamped_{index}",
                    },
                )
                smach.StateMachine.add(
                    f"LookToPoint_{index}",
                    LookToPoint(pointstamped=None),
                    transitions={
                        "succeeded": f"GetTransformedPointcloud_{index}",
                        "aborted": "failed",
                        "timed_out": f"GetTransformedPointcloud_{index}",
                    },
                    remapping={
                        "pointstamped": f"pointstamped_{index}",
                    },
                )
                if index < len(sweep_points) - 1:
                    transitions = {
                        "succeeded": f"GetPointStamped_{index+1}",
                        "failed": "failed",
                    }
                else:
                    transitions = {
                        "succeeded": "succeeded",
                        "failed": "failed",
                    }
                smach.StateMachine.add(
                    f"GetTransformedPointcloud_{index}",
                    self.GetTransformedPointcloud(),
                    transitions=transitions,
                    remapping={
                        "transformed_pointcloud": f"transformed_pointcloud_{index}",
                    },
                )

    class GetTransformedPointcloud(smach.State):
        def __init__(self, depth_topic: str = "/xtion/depth_registered/points", node: Node = None):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["transformed_pointclouds"],
                output_keys=["transformed_pointclouds"],
            )
            self.depth_topic = depth_topic
            self.node = node

        def execute(self, userdata):
            try:
                pcl = rclpy.wait_for_message(self.depth_topic, PointCloud2)   # TODO fix this
                # transform pcl to map frame
                trans = tf_buffer.lookup_transform(
                    "map",
                    pcl.header.frame_id,
                    Time(seconds=0),
                    Duration(seconds=1.0),
                )
                pcl_map = pcl_transform(pcl, trans)
                userdata.transformed_pointclouds.append(pcl_map)
            except Exception as e:
                node.get_logger().error(f"Failed to get and transform pointcloud: {str(e)}")
                return "failed"
            return "succeeded"

    class GetPointStamped(smach.State):
        def __init__(self, point: Optional[Point]):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["point"] if point is None else [],
                output_keys=["pointstamped"],
            )
            self.point = point

        def execute(self, userdata):
            try:
                if self.point is None:
                    point = userdata.point
                else:
                    point = self.point
                pointstamped = PointStamped(
                    point=point,
                    header=Header(frame_id="map"),
                )
                userdata.pointstamped = pointstamped
            except Exception as e:
                node.get_logger().error(f"Failed to create PointStamped: {str(e)}")
                return "failed"
            return "succeeded"

def main(args=None):
    global node

    rclpy.init(args=args)
    node = rclpy.create_node('pointcloud_sweep')

    sweep_points = [
        (5.78, 3.06, 0.8),
        (5.06, 3.61, 0.8),
        (4.31, 4.22, 0.8),
    ]

    try:
        sm = PointCloudSweep(sweep_points)
        sm.userdata.transformed_pointclouds = []
        outcome = sm.execute()
        node.get_logger().info(f"Pointcloud sweep completed with outcome: {outcome}")
        node.get_logger().info(
            f"Number of Transformed pointclouds: {len(sm.userdata.transformed_pointclouds)}"
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Node PointCloud Sweep")
    finally:
        rclpy.shutdown()
        node.destroy_node()

if __name__ == "__main__":
    main()
