import smach
import rospy
from tiago_controllers.helpers.pose_helpers import get_pose_from_param


class GoToWaitForPerson(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.default = default

    def execute(self, userdata):
        self.default.voice.speak("I will wait for a person")
        res = self.default.controllers.base_controller.ensure_sync_to_pose(get_pose_from_param('/wait_position/pose'))
        rospy.logerr(res)

        return 'succeeded'
