#!/usr/bin/env python3
from typing import List, Union, Optional

import rospy
import smach
import message_filters

from sensor_msgs.msg import Image, CameraInfo
from lasr_vision_msgs.srv import LangSam, LangSamRequest


class DetectLangSam3D(smach.State):

    _image_topic: str
    _depth_image_topic: str
    _depth_camera_info_topic: str
    _target_frame: str
    _box_threshold: float
    _text_threshold: float
    _prompt: Optional[str]
    _data: Union[None, tuple] = None
    _lang_sam_service: rospy.ServiceProxy
    _ts: message_filters.ApproximateTimeSynchronizer

    def __init__(
        self,
        image_topic: str = "/xtion/rgb/image_raw",
        depth_image_topic: str = "/xtion/depth_registered/image_raw",
        depth_camera_info_topic: str = "/xtion/depth_registered/camera_info",
        box_threshold: float = 0.3,
        text_threshold: float = 0.3,
        target_frame: str = "base_footprint",
        prompt: Optional[str] = None,
        slop=0.1,
    ):
        if prompt is None:
            input_keys = ["lang_sam_prompt"]
            self._prompt = None
        else:
            self._prompt = prompt
            input_keys = []
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=input_keys,
            output_keys=["lang_sam_detections_3d", "image_raw"],
        )
        self._image_topic = image_topic
        self._depth_image_topic = depth_image_topic
        self._depth_camera_info_topic = depth_camera_info_topic
        self._target_frame = target_frame
        self._box_threshold = box_threshold
        self._text_threshold = text_threshold
        self._prompt = prompt

        image_sub = message_filters.Subscriber(self._image_topic, Image)
        depth_sub = message_filters.Subscriber(self._depth_image_topic, Image)
        cam_info_sub = message_filters.Subscriber(
            self._depth_camera_info_topic, CameraInfo
        )

        self._ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub, cam_info_sub], queue_size=10, slop=slop
        )
        self._data = None

        self._lang_sam_service = rospy.ServiceProxy("/lasr_vision/lang_sam", LangSam)
        self._lang_sam_service.wait_for_service()

    def execute(self, userdata):

        def callback(image_msg, depth_msg, cam_info_msg):
            self._data = (image_msg, depth_msg, cam_info_msg)

        self._ts.registerCallback(callback)

        while not self._data:
            rospy.sleep(0.1)

        image_msg, depth_msg, cam_info_msg = self._data
        try:
            prompt = self._prompt if self._prompt else userdata.lang_sam_prompt
            request = LangSamRequest(
                image_raw=image_msg,
                depth_image=depth_msg,
                depth_camera_info=cam_info_msg,
                box_threshold=self._box_threshold,
                text_threshold=self._text_threshold,
                target_frame=self._target_frame,
                prompt=prompt,
            )
            resp = self._lang_sam_service(request)
            userdata.lang_sam_detections_3d = resp.detections
            for det in resp.detections:
                rospy.loginfo(f"Detection Point: {det.point}")
                rospy.loginfo(f"Detection Name: {det.name}")
            userdata.image_raw = image_msg
        except Exception as e:
            rospy.logerr(f"Failed LangSam Request: {e}")
            return "failed"
        return "succeeded"


if __name__ == "__main__":
    rospy.init_node("detect")
    while not rospy.is_shutdown():
        detect = DetectLangSam3D(
            slop=10.0,
            prompt="costa coffee cup. breadsticks. batteries. milk carton. banana. small plastic bottle. large plastic bottle. pringles tube.",
        )
        sm = smach.StateMachine(outcomes=["succeeded", "failed"])
        with sm:
            smach.StateMachine.add(
                "DETECT",
                detect,
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
        sm.execute()
