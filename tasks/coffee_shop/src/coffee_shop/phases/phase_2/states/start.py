import smach

class Start(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['done'])
        self.context = context
    def execute(self, userdata):
        self.context.voice_controller.sync_tts("Starting Phase 2.")
        rospy.loginfo(f"Context: {str(self.context)}")
        return 'done'
