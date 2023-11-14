import smach
import rospy
from tiago_controllers.helpers.pose_helpers import get_pose_from_param


class End(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.default = default

    def execute(self, userdata):
        guest1name = rospy.get_param('guest1/name')
        guest1drink = rospy.get_param('guest1/drink')
        guest2drink = rospy.get_param('guest2/drink')
        guest2name = rospy.get_param('guest2/name')

        self.default.voice.speak(f"{guest1name}'s favourite drink was {guest1drink}")
        self.default.voice.speak(f"{guest2name}'s favourite drink was {guest2drink}")

        return 'succeeded'


