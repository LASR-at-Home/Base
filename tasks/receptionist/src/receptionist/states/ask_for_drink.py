import smach
import rospy
from receptionist.speech_helper import get_drink

class AskForDrink(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.default = default


    def execute(self, userdata):
        
        guestcount = rospy.get_param("guestcount/count", 0)

        self.default.voice.speak("What is your favourite drink?")


        for _ in range(3):
            try:
                drink = get_drink(self.default)
            except:
                drink = "unknown    "
            if drink == "unknown":
                self.default.voice.speak("Sorry, I didn't get that. Could you repeat, please?")
            else:
                break
        
        if drink == "unknown":
            self.default.voice.speak("Sadly, I couldn't understand your favourite drink. You might go thirsty this evening.")
        else:
            self.default.voice.speak(f"{drink}, what a great drink!")

        rospy.set_param(f"guest{guestcount+1}/drink", drink)
        rospy.set_param("guestcount/count", guestcount+1)
        return 'succeeded'
