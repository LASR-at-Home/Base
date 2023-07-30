#!/usr/bin/env python3
import rospy
import actionlib
from std_msgs.msg import String

from pal_interaction_msgs.msg import TtsAction, TtsGoal, TtsText

from dialogflow_speech.msg import DialogAction, DialogGoal



class ReceptionistAPI():
    def __init__(self):
        self.dialog_client = actionlib.SimpleActionClient('/dialogflow_speech/dialogflow_speech', DialogAction)   
        self.speaking_client = actionlib.SimpleActionClient('/tts', TtsAction)
        # self.name_subscriber = rospy.Subscriber('/dialogflow_speech/RECEPTIONIST_NAME_PUBLISHER', String, self.name_cb)
        # self.favourite_drink_subscriber = rospy.Subscriber('/dialogflow_speech/RECEPTIONIST_FAVOURITE_DRINK_PUBLISHER', String, self.favourite_drink_cb)
        self.name = None
        self.favourite_drink = None

    def start_conversation(self):
        rospy.loginfo('entering method')
        self.dialog_client.wait_for_server()
        self.dialog_client.send_goal(DialogGoal('converse'))
        rospy.loginfo('spinning')
        return self.dialog_client.wait_for_result()
    
    def ask_name(self):
        self.dialog_client.wait_for_server()
        # self.speak("May I know your name")
        self.dialog_client.send_goal(DialogGoal('ask_name'))
        return self.dialog_client.wait_for_result()  


    def get_name(self):
        self.dialog_client.wait_for_server()
        # self.speak("May I know your name")
        self.dialog_client.send_goal(DialogGoal('get_name'))
        rospy.loginfo('spinning')
        self.wait_for_completion()
        self.name = rospy.get_param('/person/name')
        rospy.set_param('/person/name', '')
        return self.name
        

    def name_cb(self, data):
        self.name = data.data
        rospy.loginfo('set')

    def get_favourite_drink(self):
        self.dialog_client.wait_for_server()
        # self.speak('May I know your favourite drink?')
        self.dialog_client.send_goal(DialogGoal('get_favourite_drink'))
        rospy.loginfo('spinning')
        self.wait_for_completion()   
        
        self.favourite_drink = rospy.get_param('/person/drink')
        rospy.set_param('/person/drink', '')
        return self.favourite_drink

    def favourite_drink_cb(self, data):
        self.favourite_drink = data.data

    def ask_favourite_drink(self):
        self.dialog_client.wait_for_server()
        # self.speak('May I know your favourite drink?')
        self.dialog_client.send_goal(DialogGoal('ask_favourite_drink'))
        result = self.dialog_client.wait_for_result()
        if result:
            self.favourite_drink = self.dialog_client.get_result().name
        return self.favourite_drink


    def speak(self, message):
        self.speaking_client.wait_for_server()
        # self.speaking_client.wait_for_server(rospy.Duration(15))
        ttsgoal = TtsGoal(rawtext=TtsText(text = message, lang_id = "en" ))
        self.speaking_client.send_goal(ttsgoal)
        if self.speaking_client.wait_for_result():
            return "SUCCESS"
        else:
            rospy.logwarn("speech failure")
            return "FAILED"
        # pass


    ##################### PRIVATE METHODS ################
    def wait_for_completion(self):
        if self.dialog_client.wait_for_result():
            return "SUCCESS"
        else:
            return "FAILED"




if __name__ == "__main__":
    rospy.init_node('receptionistAPI')
    api = ReceptionistAPI()

    #api.start_conversation()    
    api.get_name()
    #api.get_favourite_drink()
    #api.speak("Hi how are you?")
    rospy.spin()


