#!/usr/bin/env python3
import rospy
from lasr_dialogflow.srv import DialogflowAudio, DialogflowAudioResponse, DialogflowText, DialogflowTextResponse
from lasr_dialogflow.actions import ReceptionistAction
import json
import os
from std_msgs.msg import String

class DialogflowServer():

    def __init__(self):
        
        self.config = {
            "robocup_receptionist" : {
                "project_id" : "robocup-receptionist-evnn",
                "cls" : ReceptionistAction
            }
        }

        self.audio_srv = rospy.Service("dialogflow_server/process_audio", DialogflowAudio, self.process_audio)
        self.text_srv = rospy.Service("dialogflow_server/process_text", DialogflowText, self.process_text)
    
    def process_audio(self, req):

        response = DialogflowAudioResponse()

        project_id = self.config[req.task.data]["project_id"]
        task = self.config[req.task.data]["cls"](project_id)
        result = task.actions[req.action.data]()

        if result:
            response.result = String(result)
            response.success = True

        return response
    
    def process_text(self, req):

        response = DialogflowTextResponse()

        project_id = self.config[req.task.data]["project_id"]
        task = self.config[req.task.data]["cls"](project_id)
        result = task.actions[req.action.data](use_mic=False, text=req.query_text.data)

        if result:
            response.result = String(result)
            response.success = True

        return response

if __name__ == "__main__":
    rospy.init_node("dialogflow_server")
    dialogflow_server = DialogflowServer()
    rospy.spin()