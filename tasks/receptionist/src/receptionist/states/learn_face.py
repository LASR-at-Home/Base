import smach
import rospy
import sys
from lasr_voice import Voice


from lasr_vision_msgs.srv import (
    LearnFace,
    LearnFaceRequest,
    LearnFaceResponse,
)


class LearnFaces(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])

    def execute(self, userdata):
        guestcount = rospy.get_param("guestcount/count", 0)

        print("here we will learn faces")

        # Here we will learn faces
        try:
            learn_service = rospy.ServiceProxy("/learn_face", LearnFace)
            req = LearnFaceRequest()
            req.name = rospy.get_param(f"guest{guestcount+1}/name", "Jane")
            req.dataset = '/home/rexy/lasr_robotclub/workspaces/haiwei_ws/src/Base/common/vision/lasr_vision_deepface/datasets'
            req.n_images = 10
            resp = learn_service(req)
        except ValueError as e:
            print("No face detected. Error:" + e)
            return 'failed'
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return 'failed'
        

        return 'succeeded'
