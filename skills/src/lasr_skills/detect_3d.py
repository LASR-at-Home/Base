#!/usr/bin/env python3
from typing import List, Union, Optional
import time

import rclpy
from rclpy.node import Node

from smach_ros import RosState
from smach import StateMachine

import message_filters

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from lasr_vision_msgs.srv import YoloDetection3D
from std_msgs.msg import String

'''
    TODO: 
        - message_filters subscribers
'''

class Detect3D(RosState):
    def __init__(
        self,
        node: Node,
        image_topic: str = "/xtion/rgb/image_raw",
        depth_image_topic: str = "/xtion/depth_registered/image_raw",
        depth_camera_info_topic: str = "/xtion/depth_registered/camera_info",
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

        image_sub = message_filters.Subscriber(self.node, self.image_topic, Image)
        depth_sub = message_filters.Subscriber(self.node, self.depth_image_topic, Image)
        cam_info_sub = message_filters.Subscriber(self.node, self.depth_camera_info_topic, CameraInfo)
        subs = [image_sub, depth_sub, cam_info_sub]
        if point_cloud_topic:
            point_cloud_sub = message_filters.Subscriber(self.node, self.point_cloud_topic, PointCloud2)
            subs.append(point_cloud_sub)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            subs, queue_size=10, slop=slop
        )
        self.data = None

        self.yolo = self.node.create_client(YoloDetection3D, "/yolo/detect3d")
        while not self.yolo.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("'YoloDetection3D' service is not available... Waiting.")

    def execute(self, userdata):

        if self.point_cloud_topic is not None:

            def callback(image_msg, depth_msg, cam_info_msg, pcl_msg):
                self.data = (image_msg, depth_msg, cam_info_msg, pcl_msg)

        else:

            def callback(image_msg, depth_msg, cam_info_msg):
                self.data = (image_msg, depth_msg, cam_info_msg)

        self.ts.registerCallback(callback)

        while not self.data:
            time.sleep(0.1)

        if len(self.data) == 4:
            image_msg, depth_msg, cam_info_msg, pcl_msg = self.data
        else:
            image_msg, depth_msg, cam_info_msg = self.data
            pcl_msg = None

        try:
            resp = self.yolo(
                image_raw=image_msg,
                depth_image=depth_msg,
                depth_camera_info=cam_info_msg,
                model=self.model,
                models=self.models,
                confidence=self.confidence,
                filter=self.filter,
                target_frame=self.target_frame,
            )
            userdata.detections_3d = resp
            userdata.image_raw = image_msg
            userdata.pcl = pcl_msg
            return "succeeded"
        except Exception as e:
            self.node.get_logger().error(f"Service call failed: {e}")
            return "failed"


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("detect")
    while not rclpy.ok():
        detect = Detect3D(node=node, slop=10.0)
        sm = StateMachine(outcomes=["succeeded", "failed"])
        with sm:
            StateMachine.add(
                "DETECT",
                detect,
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
        sm.execute()
