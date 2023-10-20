import smach
import rospy
from std_msgs.msg import Int16

class Start(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['done'])
        self.context = context

    def execute(self, userdata):
        self.context.datahub_stop_phase.publish(Int16(2))
        self.context.voice_controller.sync_tts("Starting Phase 3.")
        self.context.datahub_start_phase.publish(Int16(3))
        rospy.loginfo(f"Context: {str(self.context)}")
        self.context.voice_controller.async_tts("I am going to wait for a new customer")
        return 'done'
