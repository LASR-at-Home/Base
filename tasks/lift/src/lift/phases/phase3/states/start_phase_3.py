import smach
import rospy

class StartPhase3(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success'])
        self.default = default

    def execute(self, userdata):
        if rospy.get_param("/is_simulation"):
            rospy.loginfo(" A final update - I am starting Phase 3.")
        else:
            self.default.voice.sync_tts(" A final update - I am starting Phase 3.")

        return 'success'