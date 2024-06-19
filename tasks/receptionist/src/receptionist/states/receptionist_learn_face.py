"""The receptionist version of learn faces uses userdata for the name of the guest instead"""

import smach
import rospy
import sys


from lasr_vision_msgs.srv import (
    LearnFace,
    LearnFaceRequest,
    LearnFaceResponse,
)


class ReceptionistLearnFaces(smach.State):
    def __init__(self, guest_id: str):
        smach.State.__init__(
            self, outcomes=["succeeded", "failed"], input_keys=["guest_data"]
        )
        self._guest_id = guest_id

    def execute(self, userdata):
        print("here we will learn faces")
        try:
            learn_service = rospy.ServiceProxy("/learn_face", LearnFace)
            guest_name = userdata.guest_data[self._guest_id]["name"]
            print(guest_name)
            req = LearnFaceRequest()
            req.name = guest_name
            req.dataset = "receptionist"
            req.n_images = 10
            resp = learn_service(req)
        except ValueError as e:
            print("No face detected. Error:" + str(e))
            return "failed"
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return "failed"

        return "succeeded"
