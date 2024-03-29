import smach
import rospy
from tiago_controllers.helpers.pose_helpers import get_pose_from_param


class Start(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.default = default

    def execute(self, userdata):
        self.default.voice.speak("Start of task.")
        rospy.set_param("guest2/drink","unknown")
        rospy.set_param("guest1/drink","unknown")
        rospy.set_param("guestcount/count",0)

        res = self.default.controllers.base_controller.ensure_sync_to_pose(get_pose_from_param('/start/pose'))
        rospy.logerr(res)

        return 'succeeded'
