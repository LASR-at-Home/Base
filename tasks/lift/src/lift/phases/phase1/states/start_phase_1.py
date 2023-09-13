import smach
import rospy
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from tiago_controllers.helpers.nav_map_helpers import play_motion_goal


class StartPhase1(smach.State):
    def __init__(self, controllers, voice, pm):
        smach.State.__init__(self, outcomes=['success'])
        self.voice = voice
        self.controllers = controllers
        self.pm = pm

    def execute(self, userdata):
        res = self.controllers.base_controller.ensure_sync_to_pose(get_pose_from_param('/start/pose'))
        rospy.logerr(res)

        self.voice.speak("Hi, my name is Tiago.")
        play_motion_goal(self.pm, 'wave')
        self.voice.speak("I am the robot that knows how to take the lift.")
        self.voice.sync_tts("Nice to meet you")

        # ensure home pos
        play_motion_goal(self.pm, 'home')

        self.voice.speak("I will now start Phase 1!")


        return 'success'
