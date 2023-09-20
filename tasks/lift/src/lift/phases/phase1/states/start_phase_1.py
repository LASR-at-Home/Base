import smach
import rospy
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from tiago_controllers.helpers.nav_map_helpers import play_motion_goal
from std_msgs.msg import Int16


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
        # play_motion_goal(self.default.pm, 'wave')
        self.default.voice.speak("I am the robot that knows how to take the lift.")
        self.default.voice.sync_tts("Nice to meet you")

        # ensure home pos
        # play_motion_goal(self.default.pm, 'home')

        self.default.voice.speak("Phase 1 is really short so let's continue!")
        self.default.controllers.torso_controller.sync_reach_to(0.2)

        # self.default.datahub_start_phase.publish(Int16(1))


        return 'success'
