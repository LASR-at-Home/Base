import smach
import rospy


class Start(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=["done"])
        self.context = context

    def execute(self, userdata):
        rospy.loginfo(f"Context: {str(self.context)}")
        self.context.voice_controller.async_tts("I am going to wait for a new customer")
        return "done"
