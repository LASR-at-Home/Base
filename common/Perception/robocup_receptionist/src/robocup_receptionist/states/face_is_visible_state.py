#!/usr/bin/env python3
import rospy
import smach

from sensor_msgs.msg import Image
from open_pose.srv import DetectKeypoints
import json
from cv_bridge3 import CvBridge
from dialogflow_speech.utils import talk

class FaceVisibleState(smach.State):

    def __init__(self, torso_controller=None):
        smach.State.__init__(self, outcomes=['redetection_required', 'no_redetection_required'], input_keys=['person_bb'])

        self.torso_controller = torso_controller

        if self.torso_controller:
            self.sync_raise = self.torso_controller.sync_raise
        else:
            self.sync_raise = lambda offset : None

        self.detect_keypoints = rospy.ServiceProxy("detect_keypoints", DetectKeypoints)
        self.offset = 0.2
        self.bridge = CvBridge()
        self.risen = False

    def execute(self, userdata):
        if self.risen:
            return "no_redetection_required"
        img_msg = rospy.wait_for_message("/xtion/rgb/image_raw", Image)

        im = self.bridge.imgmsg_to_cv2_np(img_msg)
        x, y, w, h = userdata.person_bb
        im = im[y:y+h, x:x+w]

        img_msg = self.bridge.cv2_to_imgmsg(im)

        keypoints = json.loads(
            self.detect_keypoints(
                img_msg
            ).keypoints.json_keypoints.data
        )

        if (
            keypoints.get("Nose") and
            keypoints.get("R-Eye") and
            keypoints.get("L-Eye")
        ):
            return "no_redetection_required"
        else:

            talk("I can't see your face, so I will adjust my torso.")
            self.sync_raise(self.offset)
            self.offset *= -1
            self.risen = True
            return "redetection_required"