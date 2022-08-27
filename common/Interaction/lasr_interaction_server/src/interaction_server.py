#!/usr/bin/env python3
import rospy

from std_msgs.msg import String
from lasr_interaction_server.srv import SpeechInteraction, TextInteraction
from lasr_dialogflow.srv import DialogflowAudio, DialogflowText
from pal_interaction_msgs.msg import TtsActionGoal

class InteractionServer():

    def __init__(self):
        
        self.dialogflow_process_audio = rospy.ServiceProxy("/dialogflow_speech/process_audio", DialogflowAudio)
        self.dialogflow_process_text = rospy.ServiceProxy("/dialogflow_speech/process_text", DialogflowText)
    
    def speech_interaction(self, req):
        
        response = SpeechInteractionResponse()

        dialogflow_response = self.dialogflow_process_audio(req.task, req.action)
    
        response.result, response.success = dialogflow_response.result, dialogflow_response.success

        return response

    def text_interaction(self, req):

        response = SpeechInteractionResponse()

        dialogflow_response = self.dialogflow_process_text(req.task, req.action, req.query_text)
    
        response.result, response.success = dialogflow_response.result, dialogflow_response.success

        return response

    def speak(self, text):
        pass

if __name__ == "__main__":
    rospy.init_node("lasr_interaction_server")
    interaction_server = InteractionServer()
    rospy.spin()