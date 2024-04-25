import rospy
import smach
from sensor_msgs.msg import Image

from lasr_vision_msgs.srv import PointingDirection


class PointingDetector(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=["succeeded", "failed"], output_keys=["direction"]
        )
        self.service = rospy.ServiceProxy(
            "/pointing_detection_service", PointingDirection
        )
        self.image_raw = rospy.wait_for_message("/xtion/rgb/image_raw", Image)

    def execute(self, userdata):
        resp = self.service(self.image_raw)
        userdata.direction = resp.direction

        return "succeeded" if resp.direction != "Err" else "failed"
