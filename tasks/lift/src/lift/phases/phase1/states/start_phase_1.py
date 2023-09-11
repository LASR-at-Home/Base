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
        if rospy.get_param("/is_simulation"):
            rospy.loginfo("Hi, my name is Tiago and I am starting Phase 1.")
        else:
            self.voice.sync_tts("Hi, my name is Tiago and I am starting Phase 1.")
            # self.voice.sync_tts("starting Phase 1.")

        res = self.controllers.base_controller.ensure_sync_to_pose(get_pose_from_param('/start/pose'))
        rospy.logerr(res)
        print("I AM HERE IN PHASE 1")

        # ensure home pos
        self.voice.sync_tts("Nice to meet you")
        play_motion_goal(self.pm, 'wave')

        play_motion_goal(self.pm, 'home')

        return 'success'
