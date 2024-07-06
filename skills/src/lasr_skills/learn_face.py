import rospy
import smach
from lasr_vision_msgs.srv import LearnFace as LearnFaceSrv


class LearnFace(smach.State):
    def __init__(
        self,
        dataset: str,
        name: str,
        n_images: int,
    ):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

        self._dataset = dataset
        self._name = name
        self._n_images = n_images
        self._learn_face = rospy.ServiceProxy("/learn_face", LearnFaceSrv)

    def execute(self, userdata):
        try:
            result = self._learn_face(self._dataset, self._name, self._n_images)
            return "succeeded"
        except rospy.ServiceException as e:
            rospy.logwarn(f"Unable to learn face. ({str(e)})")
            return "failed"
