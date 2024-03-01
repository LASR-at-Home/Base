import smach
import rospy
from copy import deepcopy

from sensor_msgs.msg import Image

from lasr_vision_msgs.srv import Recognise, RecogniseRequest
from lasr_voice import Voice

class DetectFaces(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.default = default


    def execute(self, userdata):
        
        # Here we will learn faces
        return 'succeeded'
