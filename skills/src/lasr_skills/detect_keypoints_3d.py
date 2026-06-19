#!/usr/bin/env python3
from typing import List, Union, Optional

import rclpy

import yasmin
import yasmin_ros
from yasmin import Blackboard, StateMachine
from yasmin_ros import set_ros_loggers, ServiceState
from yasmin_viewer import YasminViewerPub

import message_filters

import time

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from lasr_vision_interfaces.srv import YoloPoseDetection3D


class DetectKeypoints3D(ServiceState):
    def __init__(
        self,
        image_topic: str = "/head_front_camera/rgb/image_raw",
        depth_image_topic: str = "/head_front_camera/depth/image_raw",
        depth_camera_info_topic: str = "/head_front_camera/depth/camera_info",
        model: str = "yolo11n-pose.pt",
        confidence: float = 0.5,
        target_frame: str = "map",
        slop=0.1,
    ):
        super().__init__(
            srv_type=YoloPoseDetection3D,
            srv_name="/yolo/detect3d_pose",
            create_request_handler=self._create_req,
            outcomes=["succeeded", "failed"],
            response_handler=self.response_handler,
        )

        self.add_output_key("keypoint_detections_3d")
        self.add_output_key("image_raw")

        self.image_topic = image_topic
        self.depth_image_topic = depth_image_topic
        self.depth_camera_info_topic = depth_camera_info_topic
        self.model = model
        self.confidence = confidence
        self.target_frame = target_frame

        self.node = yasmin_ros.logger_node

        camera_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.cam_info = None
        self.data = None
        self.image_msg = None

        image_sub = message_filters.Subscriber(
            self.node, Image, self.image_topic, qos_profile=camera_qos
        )

        depth_sub = message_filters.Subscriber(
            self.node, Image, self.depth_image_topic, qos_profile=camera_qos
        )
        cam_info_sub = message_filters.Subscriber(
            self.node, CameraInfo, self.depth_camera_info_topic, qos_profile=camera_qos
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub, cam_info_sub], queue_size=10, slop=slop
        )
        
        self.ts.registerCallback(self.callback)

    def callback(self, image_msg, depth_msg, cam_info_msg):
            self.data = (image_msg, depth_msg, cam_info_msg)

    def _create_req(self, blackboard):
        self.data = None
        self.image_msg = None

        if self.cam_info is None:
            deadline = time.time() + 5.0
            while self.cam_info is None and time.time() < deadline:
                time.sleep(0.25)
            if self.cam_info is None:
                yasmin.YASMIN_LOG_ERROR(
                    f"Timed out waiting for camera info on {self.depth_camera_info_topic}"
                )
                return "failed"

        deadline = time.time() + 30.0
        while self.data is None:
            if time.time() > deadline:
                self.node.get_logger().error(
                    f"Timed out waiting for synced rgb/depth frames. "
                    f"Check that {self.image_topic} and {self.depth_image_topic} are publishing and roughly synchronized."
                )
                return "failed"
            time.sleep(0.25)

        image_msg, depth_msg = self.data

        req = YoloPoseDetection3D.Request(
            image_raw=image_msg,
            depth_image=depth_msg,
            depth_camera_info=self.cam_info,
            model=self.model,
            confidence=self.confidence,
            target_frame=self.target_frame,
        )
        self.image_msg = image_msg

        return req

    def response_handler(self, blackboard, response):
        yasmin.YASMIN_LOG_INFO(f"Got {len(response.detections)} detections")
        for det in response.detections:
            self.node.get_logger().info(
                f"  {det.keypoint_name} at ({det.point.x:.2f}, {det.point.y:.2f}, {det.point.z:.2f})"
            )

        blackboard["keypoint_detections_3d"] = response 
        blackboard["image_raw"] = self.image_msg

        return "succeeded"


def main():
    rclpy.init()
    set_ros_loggers()

    yasmin.YASMIN_LOG_INFO("yasmin_detect3d_pose_test")
    sm = StateMachine(outcomes=["succeeded", "failed"], handle_sigint=True)
    sm.add_output_key("keypoint_detections_3d")
    sm.add_output_key("image_raw")
    sm.add_output_key("pcl")

    sm.add_state(
        "DETECT3D_POSE",
        DetectKeypoints3D(target_frame="odom"),
        transitions={"succeeded": "succeeded", "failed": "failed"},
    )
    YasminViewerPub(sm, "YASMIN_DETECT3D_CLIENT")
    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
