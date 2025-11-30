import os
from ros_state import RosState
import cv2_img
import rclpy

# from .get_image import GetImage, ROS2HelperNode


class ImageCv2ToMsg(RosState):
    """
    State for converting cv2 image to sensor Image message
    """

    def __init__(self, node):
        super().__init__(
            # self, outcomes=["succeeded", "failed"], input_keys=["img", "img_msg"], output_keys=["img_msg"]
            self,
            node,
            outcomes=["succeeded", "failed"],
            input_keys=["img"],
            output_keys=["img_msg"],
        )

    def execute(self, userdata):
        userdata.img_msg = cv2_img.cv2_img_to_msg(userdata.img)
        # print(userdata.img_msg)
        return "succeeded"
