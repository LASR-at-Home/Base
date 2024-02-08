import smach
import rospy


class SpeakDescriptions(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['succeeded','failed'], input_keys=['people'])
        self.default = default

    def execute(self, userdata):
        # for person in userdata['people']:
        #     self.default.voice.speak('I see a person')
        # self.default.voice.speak('Yes I do see you. I am trying to turn to you. But if I do not move my head, it is your own issue, not mine.')
        
        #     for feature in person['features']:
        #         if feature.label:
        #             if len(feature.colours) == 0:
        #                 self.default.voice.speak(f'They have {feature.name}.')
        #                 continue
                    
        #             self.default.voice.speak(f'They have {feature.name} and it has the colour {feature.colours[0]}')

        return 'succeeded'
