import smach
import rospy
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from receptionist.defaults import TEST, PLOT_SHOW, PLOT_SAVE, DEBUG_PATH, DEBUG, RASA
from receptionist.speech_helper import listen, affirm, get_drink
import json



class AskForDrink(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['succeeded','waitforguest2','failed'])
        self.default = default


    def execute(self,userdata):

        #So, first check if this is guest 1 or guest 2. 
        #If guest 1 is unknown, then it's guest 1 
        #If guest 1 is known, then it's guest 2. 
        guestcount = rospy.get_param("guestcount/count")
        print("guest count:", guestcount)

        if (guestcount==0):
            drink = rospy.get_param("guest1/drink")
            #If we don't know, we start asking:
            if drink == "unknown":
                
                #Gazing correctly: for now, we just look ahead. 
                self.default.controllers.head_controller.look_straight()

                #Asking for the drink: 
                self.default.voice.speak("What is your favourite drink?")
            
                #Excuting whisper and rasa to get answer: 
                try: 
                    drink = get_drink(self.default)
                except: 
                    drink = "unknown"
                

                if (drink == "unknown"):
                    self.default.voice.speak("Sorry, that made no sense")
                    return 'failed'
                else: 
                    #Explain drink back: 
                    self.default.voice.speak("So, I heard your favourite drink was {}".format(drink))
                    rospy.set_param("guest1/drink",drink)
                    rospy.set_param("guestcount/count",1)
                    return 'waitforguest2'
        elif(guestcount==1):
                drink = rospy.get_param("guest2/drink")
                print("guest 2 drink: ", drink)
            #If we don't know, we start asking:
                if drink == "unknown":
                    #Gazing correctly: for now, we just look ahead. 
                    self.default.controllers.head_controller.look_straight()

                    #Asking for the drink: 
                    self.default.voice.speak("What is your favourite drink?")
            
                    #Excuting whisper and rasa to get answer: 
                    try: 
                        drink = get_drink(self.default)
                    except: 
                        drink = "unknown"
                
                    if (drink == "unknown"):
                        self.default.voice.speak("Sorry, that made no sense")
                        return 'failed'
                    else: 
                        #Explain drink back: 
                        self.default.voice.speak("So, I heard your favourite drink was {}".format(drink))
                        rospy.set_param("guest2/drink",drink)
                        rospy.set_param("guestcount/count",2)
                        return 'succeeded'
        else: 
            self.default.voice.speak("What is going on")



