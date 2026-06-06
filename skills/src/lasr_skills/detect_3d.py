#!/usr/bin/env python3
from typing import List, Union, Optional
import time

import rclpy
from rclpy.node import Node

from smach_ros import RosState
from smach import StateMachine

import message_filters

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from lasr_vision_interfaces.srv import YoloDetection3D


class Detect3D(RosState):
    def __init__(
        self,
        node: Node,
        image_topic: str = "/head_front_camera/rgb/image_raw",
        depth_image_topic: str = "/head_front_camera/depth/image_raw",
        depth_camera_info_topic: str = "/head_front_camera/depth/camera_info",
        model: str = "yolo11n-seg.pt",
        filter: Union[List[str], None] = None,
        confidence: float = 0.5,
        target_frame: str = "map",
        slop=0.2,
    ):
        RosState.__init__(
            self,
            node,
            outcomes=["succeeded", "failed"],
            output_keys=["detections_3d", "image_raw"],
        )
        self.image_topic = image_topic
        self.depth_image_topic = depth_image_topic
        self.depth_camera_info_topic = depth_camera_info_topic
        self.model = model
        self.filter = filter or []
        self.confidence = confidence
        self.target_frame = target_frame

        camera_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.cam_info = None
        self.node.create_subscription(
            CameraInfo,
            self.depth_camera_info_topic,
            self._cache_camera_info,
            qos_profile=camera_qos,
        )

        image_sub = message_filters.Subscriber(
            self.node, Image, self.image_topic, qos_profile=camera_qos
        )
        depth_sub = message_filters.Subscriber(
            self.node, Image, self.depth_image_topic, qos_profile=camera_qos
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub], queue_size=30, slop=slop
        )
        self.data = None

        self.yolo = self.node.create_client(YoloDetection3D, "/yolo/detect3d")
        while not self.yolo.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                "'YoloDetection3D' service is not available... Waiting."
            )

    def _cache_camera_info(self, msg: CameraInfo) -> None:
        if self.cam_info is None:
            self.cam_info = msg

    def execute(self, userdata):
        if self.cam_info is None:
            deadline = time.time() + 5.0
            while self.cam_info is None and time.time() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.cam_info is None:
                self.node.get_logger().error(
                    f"Timed out waiting for camera info on {self.depth_camera_info_topic}"
                )
                return "failed"

        self.data = None

        def callback(image_msg, depth_msg):
            if self.data is not None:
                return
            self.data = (image_msg, depth_msg, self.cam_info)

        self.ts.registerCallback(callback)

        deadline = time.time() + 30.0
        while not self.data:
            if time.time() > deadline:
                self.node.get_logger().error(
                    f"Timed out waiting for synced rgb/depth frames. "
                    f"Check that {self.image_topic} and {self.depth_image_topic} are publishing and roughly synchronized."
                )
                return "failed"
            rclpy.spin_once(self.node, timeout_sec=0.1)

        image_msg, depth_msg, cam_info_msg = self.data

        try:
            request = YoloDetection3D.Request(
                image_raw=image_msg,
                depth_image=depth_msg,
                depth_camera_info=cam_info_msg,
                model=self.model,
                confidence=self.confidence,
                filter=self.filter,
                target_frame=self.target_frame,
            )
            future = self.yolo.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)

            resp = future.result()

            self.node.get_logger().info(f"Got {len(resp.detected_objects)} detections")
            for det in resp.detected_objects:
                self.node.get_logger().info(
                    f"  {det.name} at ({det.point.x:.2f}, {det.point.y:.2f}, {det.point.z:.2f})"
                )

            userdata.detections_3d = resp
            userdata.image_raw = image_msg
            return "succeeded"
        except Exception as e:
            self.node.get_logger().error(f"Service call failed: {e}")
            return "failed"


def main():
    rclpy.init()
    node = rclpy.create_node("detect")
    detect = Detect3D(node=node, slop=10.0, filter=["person"])
    sm = StateMachine(outcomes=["succeeded", "failed"])
    with sm:
        StateMachine.add(
            "DETECT",
            detect,
            transitions={"succeeded": "succeeded", "failed": "failed"},
        )
    sm.execute()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
