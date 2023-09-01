import smach
import rospy

class StartPhase1(smach.State):
    def __init__(self, controllers, voice):
        smach.State.__init__(self, outcomes=['success'])
        self.voice = voice

    def execute(self, userdata):
        if rospy.get_param("/is_simulation"):
            rospy.loginfo("Hi, my name is Tiago and I am starting Phase 1.")
        else:
            self.voice.sync_tts("Hi, my name is Tiago and I am starting Phase 1.")

        return 'success'