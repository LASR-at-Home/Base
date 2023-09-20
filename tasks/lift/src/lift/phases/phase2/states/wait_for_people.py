#!/usr/bin/env python3
import smach, os, rospy
from sensor_msgs.msg import Image
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
import json
from interaction_module.srv import AudioAndTextInteraction, AudioAndTextInteractionRequest, \
    AudioAndTextInteractionResponse
from lift.defaults import TEST, PLOT_SHOW, PLOT_SAVE, DEBUG_PATH, DEBUG, RASA
from sensor_msgs.msg import PointCloud2
from lasr_object_detection_yolo.detect_objects_v8 import detect_objects, perform_detection, debug

class WaitForPeople(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.default = default


    def listen(self):
        resp = self.default.speech()
        if not resp.success:
            self.default.voice.speak("Sorry, I didn't get that")
            return self.listen()
        resp = json.loads(resp.json_response)
        rospy.loginfo(resp)
        return resp


    def get_people_number(self):
        resp = self.listen()
        if resp["intent"]["name"] != "negotiate_lift":
            self.default.voice.speak("Sorry, I misheard you, could you say again how many people?")
            return self.get_people_number()
        people = resp["entities"].get("people",[])
        if not people: 
            self.default.voice.speak("Sorry, could you say again how many people?")
            return self.get_people_number()
        people_number = int(people[0]["value"])        
        self.default.voice.speak("I hear that there are {} people".format(people_number))
        return people_number

    def safe_seg_info(self, detections):
        pos_people = []
        for i, person in detections:
            person = person.tolist()
            pos_people.append([person[0], person[1]])

        num_people = len(detections)

        rospy.set_param("/lift/num_people", num_people)
        rospy.set_param("/lift/pos_persons", pos_people)

    def affirm(self):
        # Listen to person:
        resp = self.listen()
        # Response in intent can either be yes or no.
        # Making sure that the response belongs to "affirm", not any other intent:
        if resp['intent']['name'] != 'affirm':
            self.default.voice.speak("Sorry, I didn't get that, please say yes or no")
            return self.affirm()
        choices = resp["entities"].get("choice", None)
        if choices is None:
            self.default.voice.speak("Sorry, I didn't get that")
            return self.affirm()
        choice = choices[0]["value"].lower()
        if choice not in ["yes", "no"]:
            self.default.voice.speak("Sorry, I didn't get that")
            return self.affirm()
        return choice


    def execute(self, userdata):
        # wait and ask
        rospy.set_param("/from_schedule", False)
        self.default.voice.speak("Exciting stuff, we are going to the lift!")

        count = 2
        if RASA:
            try:
                count = self.get_people_number()
            except Exception as e:
                print(e)
                count = 2
                self.default.voice.speak("I couldn't hear how many people, so I'm going to guess 2")

        self.default.voice.speak("I will wait a bit here until you go inside. You see, I am a very very good robot!")
        rospy.sleep(5)

        self.default.voice.speak("I will now move to the center of the lift waiting area")
        state = self.default.controllers.base_controller.ensure_sync_to_pose(get_pose_from_param('/wait_in_front_lift_centre/pose'))

        polygon = rospy.get_param('test_lift_points')
        pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        detections, im = perform_detection(self.default, pcl_msg, None, ["person"], "yolov8n-seg.pt")
        print("len detections")
        print(len(detections))


        self.safe_seg_info(detections)


        return 'success'



