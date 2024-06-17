import rospy
import smach

from lasr_vision_msgs.srv import Recognise as RecogniseSrv
from sensor_msgs.msg import Image


class Recognise(smach.State):
    def __init__(
        self,
        dataset: str,
        confidence: float = 0.5,
        image_topic: str = "/xtion/rgb/image_raw",
    ):
        smach.State.__init__(
            self, outcomes=["succeeded", "failed"], output_keys=["detections"]
        )
        self._dataset = dataset
        self._confidence = confidence
        self._image_topic = image_topic
        self._recognise = rospy.ServiceProxy("/recognise", RecogniseSrv)

    def execute(self, userdata):
        img_msg = rospy.wait_for_message(self._image_topic, Image)
        try:
            result = self._recognise(img_msg, self._dataset, self._confidence)
            userdata.detections = result
            return "succeeded"
        except rospy.ServiceException as e:
            rospy.logwarn(f"Unable to perform inference. ({str(e)})")
            return "failed"
