import smach
import rospy
from std_msgs.msg import Int16

class Start(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['done'])
        self.context = context
    def execute(self, userdata):
        self.context.voice_controller.sync_tts("Starting Phase 1.")
        self.context.datahub_start_phase.publish(Int16(1))
        rospy.loginfo(f"Context: {str(self.context)}")
        return 'done'
