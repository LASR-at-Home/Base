import smach
import rospy
from tiago_controllers.helpers.pose_helpers import get_pose_from_param


class WaitForPerson(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success'])
        self.default = default

    def execute(self, userdata):
        self.default.voice.speak("I'm waiting for a person.")
        self.default.controllers.torso_controller.sync_reach_to(0.25)
        res = self.default.controllers.base_controller.ensure_sync_to_pose(get_pose_from_param('/start/pose'))
        rospy.logerr(res)

        return 'success'
