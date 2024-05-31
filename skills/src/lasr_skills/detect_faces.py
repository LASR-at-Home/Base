import rospy
import smach

from lasr_vision_msgs.srv import DetectFaces as DetectFacesSrv
from sensor_msgs.msg import Image
from cv2_pcl import pcl_to_img_msg


class DetectFaces(smach.State):
    def __init__(
        self,
        image_topic: str = "/xtion/rgb/image_raw",
    ):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["pcl_msg"],
            output_keys=["detections"],
        )
        self.img_msg = None
        self._image_topic = image_topic
        self._detect_faces = rospy.ServiceProxy("/detect_faces", DetectFacesSrv)

    def execute(self, userdata):
        self.img_msg = pcl_to_img_msg(userdata.pcl_msg)
        if not self.img_msg:
            self.img_msg = rospy.wait_for_message(self._image_topic, Image)
        try:
            result = self._detect_faces(self.img_msg)
            userdata.detections = result
            return "succeeded"
        except rospy.ServiceException as e:
            rospy.logwarn(f"Unable to perform inference. ({str(e)})")
            return "failed"
