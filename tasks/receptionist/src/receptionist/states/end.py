import smach
import rospy
from tiago_controllers.helpers.pose_helpers import get_pose_from_param


class End(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.default = default

    def execute(self, userdata):
        guest1drink = rospy.get_param('guest1/drink')
        guest2drink = rospy.get_param('guest2/drink')


        self.default.voice.speak("Guest 1 favourite drink was {}".format(guest1drink))
        self.default.voice.speak("Guest 2 favourite drink was {}".format(guest2drink))


        print(guest1drink)
        print(guest2drink)
        return 'succeeded'


