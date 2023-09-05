import smach
import rospy

class StartPhase3(smach.State):
    def __init__(self, controllers, voice):
        smach.State.__init__(self, outcomes=['success'])
        self.voice = voice

    def execute(self, userdata):
        if rospy.get_param("/is_simulation"):
            rospy.loginfo(" A final update - I am starting Phase 3.")
        else:
            self.voice.sync_tts(" A final update - I am starting Phase 3.")

        return 'success'