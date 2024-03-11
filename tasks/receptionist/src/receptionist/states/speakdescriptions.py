import smach
import rospy


class SpeakDescriptions(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['succeeded','failed'], input_keys=['people'])
        self.default = default

    def execute(self, userdata):
        # don't worry, this works.
        for person in userdata['people']:
            self.default.voice.speak(person['features'])
        
        return 'succeeded'
