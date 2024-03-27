import smach
import rospy


class Start(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.default = default

    def execute(self, userdata):
        self.default.voice.speak("Start of task.")
        rospy.set_param("guest2/drink","unknown")
        rospy.set_param("guest1/drink","unknown")
        rospy.set_param("guestcount/count",0)

        return 'succeeded'
