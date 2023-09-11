#!/usr/bin/env python3
import smach, os, rospy
from sensor_msgs.msg import Image
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
import json

class WaitForPeople(smach.State):
    def __init__(self, controllers, voice, yolo,speech):
        smach.State.__init__(self, outcomes=['success', 'failed'])

        self.controllers = controllers
        self.voice = voice
        self.yolo = yolo
        self.speech = speech

    def listen(self):
        resp = self.speech()
        if not resp.success:
            self.voice.speak("Sorry, I didn't get that")
            return self.listen()
        resp = json.loads(resp.json_response)
        rospy.loginfo(resp)
        return resp


    def get_people_number(self):
        resp = self.listen()
        if resp["intent"]["name"] != "negotiate_lift":
            self.voice.speak("Sorry, I misheard you, could you say again how many people?")
            return self.get_people_number()
        people = resp["entities"].get("people",[])
        if not people: 
            self.voice.speak("Sorry, could you say again how many people?")
            return self.get_people_number()
        people_number = int(people[0]["value"])        
        self.voice.speak("I hear that there are {} people".format(people_number))
        return people_number


    def execute(self, userdata):
        # wait and ask
        self.voice.speak("How many people are thinking to go in the lift?")

        count = 2 
        try: 
            count = self.get_people_number()
        except: 
            count = 2 
            self.voice.speak("I couldn't hear how many people, so I'm going to guess 2")
         # get the answer

        result = self.controllers.base_controller.sync_to_pose(get_pose_from_param('/wait_centre/pose'))
        rospy.sleep(1)

        # send request - image, dataset, confidence, nms
        image = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        detections = self.yolo(image, "yolov8n.pt", 0.3, 0.3)

        # segment them as well and count them
        count_people = 0
        count_people = sum(1 for det in detections.detected_objects if det.name == "person")

        self.voice.speak("I see {} people".format(count_people))

        if count_people < count:
            return 'failed'
        else:
            return 'success'


        # check if they are static with the frames
