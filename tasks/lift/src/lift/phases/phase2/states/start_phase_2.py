#!/usr/bin/env python3
import smach
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
class StartPhase2(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success'])
        self.default = default

    def execute(self, userdata):
        # self.default.datahub_stop_phase.publish(Int16(1))
        self.default.voice.speak("Quickest update ever... I am starting Phase 2. I am going to the lift.")

        self.default.controllers.base_controller.sync_to_pose(get_pose_from_param('/phase2/pose'))

        return 'success'