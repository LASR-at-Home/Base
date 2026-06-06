#!/usr/bin/env python3
from typing import List, Union, Optional
import time

import rclpy
from rclpy.node import Node

from smach_ros import RosState
from smach import StateMachine

import message_filters

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from lasr_vision_interfaces.srv import YoloDetection3D


class Detect3D(RosState):
    def __init__(
        self,
        node: Node,
        image_topic: str = "/head_front_camera/rgb/image_raw",
        depth_image_topic: str = "/head_front_camera/depth/image_raw",
        depth_camera_info_topic: str = "/head_front_camera/depth/camera_info",
        point_cloud_topic: Optional[str] = None,
        model: str = "yolo11n-seg.pt",
        models: Union[List[str], None] = None,
        filter: Union[List[str], None] = None,
        confidence: float = 0.5,
        target_frame: str = "map",
        slop=1.0,
    ):
        RosState.__init__(
            self,
            node,
            outcomes=["succeeded", "failed"],
            output_keys=["detections_3d", "image_raw", "pcl"],
        )
        self.image_topic = image_topic
        self.depth_image_topic = depth_image_topic
        self.depth_camera_info_topic = depth_camera_info_topic
        self.point_cloud_topic = point_cloud_topic
        self.model = model
        self.models = models
        self.filter = filter or []
        self.confidence = confidence
        self.target_frame = target_frame

        camera_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        cam_info_sub = message_filters.Subscriber(
            self.node, CameraInfo, self.depth_camera_info_topic, qos_profile=camera_qos
        )
        # CameraInfo is latched: keep it in a cache, not in the time synchronizer.
        self.cam_info_cache = message_filters.Cache(cam_info_sub, 10)

        image_sub = message_filters.Subscriber(
            self.node, Image, self.image_topic, qos_profile=camera_qos
        )
        depth_sub = message_filters.Subscriber(
            self.node, Image, self.depth_image_topic, qos_profile=camera_qos
        )
        subs = [image_sub, depth_sub]
        if self.point_cloud_topic is not None:
            point_cloud_sub = message_filters.Subscriber(
                self.node, PointCloud2, self.point_cloud_topic, qos_profile=camera_qos
            )
            subs.append(point_cloud_sub)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            subs, queue_size=10, slop=slop
        )
        self.data = None

        self.yolo = self.node.create_client(YoloDetection3D, "/yolo/detect3d")
        while not self.yolo.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                "'YoloDetection3D' service is not available... Waiting."
            )

    def execute(self, userdata):
        self.data = None

        if self.point_cloud_topic is not None:

            def callback(image_msg, depth_msg, pcl_msg):
                if self.data is not None:
                    return
                cam_info_msg = self.cam_info_cache.getLast()
                if cam_info_msg is None:
                    return
                self.data = (image_msg, depth_msg, cam_info_msg, pcl_msg)

        else:

            def callback(image_msg, depth_msg):
                if self.data is not None:
                    return
                cam_info_msg = self.cam_info_cache.getLast()
                if cam_info_msg is None:
                    return
                self.data = (image_msg, depth_msg, cam_info_msg)

        self.ts.registerCallback(callback)

        deadline = time.time() + 30.0
        while not self.data:
            if time.time() > deadline:
                if self.cam_info_cache.getLast() is None:
                    self.node.get_logger().error(
                        f"Timed out waiting for camera info on {self.depth_camera_info_topic}"
                    )
                else:
                    self.node.get_logger().error(
                        "Timed out waiting for synced rgb/depth frames"
                    )
                return "failed"
            rclpy.spin_once(self.node, timeout_sec=0.1)

        if len(self.data) == 4:
            image_msg, depth_msg, cam_info_msg, pcl_msg = self.data
        else:
            image_msg, depth_msg, cam_info_msg = self.data
            pcl_msg = None

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
            userdata.pcl = pcl_msg
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
