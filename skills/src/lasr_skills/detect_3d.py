#!/usr/bin/env python3
from typing import List, Union

import rospy
import smach
import message_filters

from sensor_msgs.msg import Image, CameraInfo
from lasr_vision_msgs.srv import YoloDetection3D
from std_msgs.msg import String


class Detect3D(smach.State):
    def __init__(
        self,
        image_topic: str = "/xtion/rgb/image_raw",
        depth_image_topic: str = "/xtion/depth_registered/image_raw",
        depth_camera_info_topic: str = "/xtion/depth_registered/camera_info",
        model: str = "yolo11n-seg.pt",
        filter: Union[List[str], None] = None,
        confidence: float = 0.5,
        target_frame: str = "map",
    ):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["detections_3d"],
        )
        self.image_topic = image_topic
        self.depth_image_topic = depth_image_topic
        self.depth_camera_info_topic = depth_camera_info_topic
        self.model = model
        self.filter = filter or []
        self.confidence = confidence
        self.target_frame = target_frame

        image_sub = message_filters.Subscriber(self.image_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_image_topic, Image)
        cam_info_sub = message_filters.Subscriber(
            self.depth_camera_info_topic, CameraInfo
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub, cam_info_sub], queue_size=10, slop=0.1
        )
        self.data = None

        self.yolo = rospy.ServiceProxy("/yolo/detect3d", YoloDetection3D)
        self.yolo.wait_for_service()

    def execute(self, userdata):

        def callback(image_msg, depth_msg, cam_info_msg):
            self.data = (image_msg, depth_msg, cam_info_msg)

        self.ts.registerCallback(callback)

        while not self.data:
            rospy.sleep(0.1)

        image_msg, depth_msg, cam_info_msg = self.data

        try:
            resp = self.yolo(
                image_raw=image_msg,
                depth_image=depth_msg,
                depth_camera_info=cam_info_msg,
                model=self.model,
                confidence=self.confidence,
                filter=self.filter,
                target_frame=self.target_frame,
            )
            userdata.detections_3d = resp
            return "succeeded"
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return "failed"


if __name__ == "__main__":
    rospy.init_node("detect")
    while not rospy.is_shutdown():
        detect = Detect3D()
        sm = smach.StateMachine(outcomes=["succeeded", "failed"])
        with sm:
            smach.StateMachine.add(
                "DETECT",
                detect,
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
        sm.execute()
