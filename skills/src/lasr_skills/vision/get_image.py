import os
import smach
import rospy
from sensor_msgs.msg import Image


class GetImage(smach.State):
    """
    State for reading an sensor_msgs Image message
    """

    def __init__(self, topic: str = None):
        smach.State.__init__(
            self, outcomes=['succeeded', 'failed'], output_keys=['img_msg'])

        if topic is None:
            self.topic = '/xtion/rgb/image_raw' if 'tiago' in os.environ['ROS_MASTER_URI'] else '/camera/image_raw'
        else:
            self.topic = topic

    def execute(self, userdata):
        try:
            userdata.img_msg = rospy.wait_for_message(
                self.topic, Image)
        except Exception as e:
            print(e)
            return 'failed'

        return 'succeeded'
