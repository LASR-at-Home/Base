import smach
import rospy
from tiago_controllers.helpers.pose_helpers import get_pose_from_param


class StartPhase1(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success'])
        self.default = default

    def execute(self, userdata):
        self.default.voice.speak("Starting Phase 1.")
        self.default.controllers.torso_controller.sync_reach_to(0.25)
        res = self.default.controllers.base_controller.ensure_sync_to_pose(get_pose_from_param('/starting/pose'))
        rospy.logerr(res)

        self.default.voice.speak("Hi, my name is Tiago.")
        self.default.voice.speak("I am the robot that knows how to take the lift.")
        self.default.voice.sync_tts("Nice to meet you")

        self.default.voice.speak("Phase 1 is really short so let's continue!")
        self.default.controllers.torso_controller.sync_reach_to(0.2)



        return 'success'
