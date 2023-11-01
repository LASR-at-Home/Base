import smach
import rospy
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from lift.defaults import TEST, PLOT_SHOW, PLOT_SAVE, DEBUG_PATH, DEBUG, RASA
from lift.speech_helper import listen, affirm, get_drink
import json



class AskForDrink(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success'])
        self.default = default

    def execute(self, userdata):

        #Get if we know the drink of the person:
        drink = rospy.get_param("guest1/drink")

        #If we don't know, we start asking:
        if drink == "unknown":
            

            #Gazing correctly: for now, we just look ahead. 
            self.default.controllers.head_controller.look_straight()

            #Asking for the drink: 
            self.default.voice.speak("Hi, what is your favourite drink?")
        
            #Excuting whisper and rasa to get answer: 
            try: 
                drink = get_drink(self.default)
            except: 
                drink = "unknown"


            #Explain drink back: 
            self.default.voice.speak("So, I heard your favourite drink was {}".format(drink))
        return 'success'
