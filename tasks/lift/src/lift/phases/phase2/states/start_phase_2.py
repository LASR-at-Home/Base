#!/usr/bin/env python3
import smach
from tiago_controllers.helpers.nav_map_helpers import play_motion_goal

class StartPhase2(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success'])
        self.default = default

    def execute(self, userdata):
        self.default.voice.speak("Quickest update ever... I am starting Phase 2. I am going to the lift.")

        # ensure home pos
        # play_motion_goal(self.pm, 'home')

        return 'success'