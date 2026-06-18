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
from lasr_vision_interfaces.srv import YoloDetection3D

"""
    TODO: 
        - message_filters subscribers
"""


class Detect3D(ServiceState):
    def __init__(
        self,
        image_topic: str = "/head_front_camera/rgb/image_raw",
        depth_image_topic: str = "/head_front_camera/depth/image_raw",
        depth_camera_info_topic: str = "/head_front_camera/depth/camera_info",
        model: str = "yolo11n-seg.pt",
        filter: Union[List[str], None] = None,
        confidence: float = 0.5,
        target_frame: str = "map",
    ):
        super().__init__(
            srv_type=YoloDetection3D,
            srv_name="/yolo/detect3d",
            create_request_handler=self._create_req,
            outcomes=["succeeded", "failed"],
            response_handler=self.response_handler,
        )
        self.set_description("Detects 3d objects using yolo")
        self.add_output_key("detections_3d")
        self.add_output_key("image_raw")

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

        self.data = None
        self.image_msg = None

        cam_info = message_filters.Subscriber(
            self._node, CameraInfo, self.depth_camera_info_topic, qos_profile=camera_qos
        )

        self.cache = message_filters.Cache(cam_info)

        image_sub = message_filters.Subscriber(
            self._node, Image, self.image_topic, qos_profile=camera_qos
        )

        depth_sub = message_filters.Subscriber(
            self._node, Image, self.depth_image_topic, qos_profile=camera_qos
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub], queue_size=10, slop=0.1
        )

        self.ts.registerCallback(self.callback)

    def callback(self, image_msg, depth_msg):
        if self.data is None:
            self.data = (image_msg, depth_msg)

    def _create_req(self, blackboard):
        self.data = None
        self.image_msg = None

        deadline = time.time() + 30.0
        while self.data is None:
            if time.time() > deadline:
                yasmin.YASMIN_LOG_ERROR(
                    f"Timed out waiting for synced rgb/depth frames. "
                    f"Check that {self.image_topic} and {self.depth_image_topic} are publishing and roughly synchronized."
                )
                return "failed"
            time.sleep(0.25)

        image_msg, depth_msg = self.data

        req = YoloDetection3D.Request(
            image_raw=image_msg,
            depth_image=depth_msg,
            depth_camera_info=self.cache.getLast(),
            model=self.model,
            confidence=self.confidence,
            filter=self.filter,
            target_frame=self.target_frame,
        )
        self.image_msg = image_msg

        return req

    def response_handler(self, blackboard, response):
        yasmin.YASMIN_LOG_INFO(f"Got {len(response.detected_objects)} detections")
        for det in response.detected_objects:
            yasmin.YASMIN_LOG_INFO(
                f"  {det.name} at ({det.point.x:.2f}, {det.point.y:.2f}, {det.point.z:.2f})"
            )

        blackboard["detections_3d"] = response
        blackboard["image_raw"] = self.image_msg

        return "succeeded"


def main():
    rclpy.init()
    set_ros_loggers()

    yasmin.YASMIN_LOG_INFO("yasmin_detect3d_test")
    sm = StateMachine(outcomes=["succeeded", "failed"], handle_sigint=True)
    sm.add_output_key("detections_3d")
    sm.add_output_key("image_raw")
    sm.add_output_key("pcl")

    sm.add_state(
        "DETECT3D",
        Detect3D(target_frame="odom"),
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