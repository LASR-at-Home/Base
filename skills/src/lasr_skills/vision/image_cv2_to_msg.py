import os
import smach
import cv2_img


class ImageCv2ToMsg(smach.State):
    """
    State for converting cv2 image to sensor Image message
    """

    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succeeded'], input_keys=['img'], output_keys=['img_msg'])

    def execute(self, userdata):
        userdata.img_msg = cv2_img.cv2_img_to_msg(userdata.img)
        return 'succeeded'
