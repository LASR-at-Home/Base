#!/usr/bin/env python3
import smach
from tiago_controllers.helpers.nav_map_helpers import play_motion_goal
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from std_msgs.msg import Int16
class StartPhase2(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success'])
        self.default = default

    def execute(self, userdata):
        # self.default.datahub_stop_phase.publish(Int16(1))
        self.default.voice.speak("Quickest update ever... I am starting Phase 2. I am going to the lift.")

        # ensure home pos
        # play_motion_goal(self.pm, 'home')
        # self.default.datahub_start_phase.publish(Int16(2))

        self.default.controllers.base_controller.sync_to_pose(get_pose_from_param('/phase2/pose'))

        return 'success'