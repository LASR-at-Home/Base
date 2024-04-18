import os
import smach
import cv2_img


class ImageMsgToCv2(smach.State):
    """
    State for converting a sensor Image message to cv2 format
    """

    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succeeded'], input_keys=['img_msg'], output_keys=['img'])

    def execute(self, userdata):
        userdata.img = cv2_img.msg_to_cv2_img(userdata.img_msg)
        return 'succeeded'
