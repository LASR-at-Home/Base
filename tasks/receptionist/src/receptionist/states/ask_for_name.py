import smach
import rospy
from receptionist.speech_helper import get_name

class AskForName(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.default = default


    def execute(self, userdata):
        
        guestcount = rospy.get_param("guestcount/count", 0)

        self.default.voice.speak("What is your name?")

        for _ in range(3):
            try:
                name = get_name(self.default)
            except:
                name = "unknown"
            if name == "unknown":
                self.default.voice.speak("Sorry, I didn't get that. Could you repeat, please?")
            else:
                break
        
        if name == "unknown":
            self.default.voice.speak("I couldn't understand your name, but it's great to meet you!")
        else:
            self.default.voice.speak(f"It's great to meet you {name}!")

        rospy.set_param(f"guest{guestcount+1}/name", name)
        return 'succeeded'
