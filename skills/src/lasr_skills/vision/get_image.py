import os
import smach
import rospy
from sensor_msgs.msg import Image, PointCloud2


class GetImage(smach.State):
    """
    State for reading an sensor_msgs Image message
    """

    def __init__(self, topic: str = None):
        smach.State.__init__(
            self, outcomes=['succeeded', 'failed'], output_keys=['img_msg'])

        if topic is None:
            self.topic = '/xtion/rgb/image_raw' if 'tiago' in os.environ['ROS_MASTER_URI'] else '/usb_cam/image_raw'
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


class Get3DImage(smach.State):
    """
    State for acquiring a PointCloud2 message.
    """

    def __init__(self, topic: str = None):
        smach.State.__init__(
            self, outcomes=['succeeded', 'failed'], output_keys=['img_msg'])

        if topic is None:
            self.topic = '/xtion/depth_registered/points'
        else:
            self.topic = topic

    def execute(self, userdata):
        try:
            userdata.img_msg = rospy.wait_for_message(
                self.topic, PointCloud2)
        except Exception as e:
            print(e)
            return 'failed'

        return 'succeeded'
