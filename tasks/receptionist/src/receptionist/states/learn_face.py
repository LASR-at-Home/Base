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
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.default = default

        self.dataset = sys.argv[2]

      #  rospy.init_node("face_learner", anonymous=True)
        # rospy.wait_for_service("/learn_face")
       # rospy.spin()


    def execute(self, userdata):
        guestcount = rospy.get_param("guestcount/count", 0)

        print("here we will learn faces")
        self.default.voice.speak("I'm about to learn your face")

        # Here we will learn faces
        try:
            learn_service = rospy.ServiceProxy("/learn_face", LearnFace)
            req = LearnFaceRequest()
            req.name = rospy.get_param(f"guest{guestcount}/name", "Jane")
            # rospy.loginfo(f"gurstcount: {guestcount}")
            # rospy.loginfo(f"guest{guestcount}/name")
            # rospy.loginfo(f"name received: {req.name}")
            req.dataset = '/home/rexy/Documents/robotclub/robocup_ws/src/base_zoe_fork/common/vision/lasr_vision_deepface/datasets'
            req.n_images = 10
            resp = learn_service(req)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return 'failed'

        return 'succeeded'
