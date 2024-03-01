#!/usr/bin/env python3

import smach
import rospy
import sys

from lasr_vision_msgs.srv import (
    LearnFace,
    LearnFaceRequest,
    LearnFaceResponse,
)


class LearnFaces(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.default = default

        self.dataset = sys.argv[2]

        rospy.init_node("face_learner", anonymous=True)
        # rospy.wait_for_service("/learn_face")
        rospy.spin()


    def execute(self, userdata):
        
        # Here we will learn faces
        try:
            learn_service = rospy.ServiceProxy("/learn_face", LearnFace)
            req = LearnFaceRequest()
            req.name = "Jane"
            req.dataset = self.dataset
            req.n_images = 10
            resp = learn_service(req)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return 'failed'
        return 'succeeded'
