import smach
import rospy


class SpeakDescriptions(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['succeeded','failed'], input_keys=['people'])
        self.default = default

    def execute(self, userdata):
        for person in userdata['people']:
            self.default.voice.speak(person['features'])
            break  # only speak for the first person in the frame
        
        return 'succeeded'
