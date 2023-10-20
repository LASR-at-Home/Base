#!/usr/bin/env python3
import smach, os, rospy
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
import json
from lift.defaults import TEST, PLOT_SHOW, PLOT_SAVE, DEBUG_PATH, DEBUG, RASA
from sensor_msgs.msg import PointCloud2
from lasr_object_detection_yolo.detect_objects_v8 import detect_objects, perform_detection, debug
from speech_helper import listen, affirm, hear_wait, get_people_number



class WaitForPeople(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.default = default


    def safe_seg_info(self, detections):
        pos_people = []
        for i, person in detections:
            person = person.tolist()
            pos_people.append([person[0], person[1]])

        num_people = len(detections)

        rospy.set_param("/lift/num_people", num_people)
        rospy.set_param("/lift/pos_persons", pos_people)

    def execute(self, userdata):
        # wait and ask
        rospy.set_param("/from_schedule", False)
        self.default.voice.speak("Exciting stuff, we are going to the lift!")

        # count = 2
        # if RASA:
        #     try:
        #         count = get_people_number(self.default)
        #     except Exception as e:
        #         print(e)
        #         count = 2
        #         self.default.voice.speak("I couldn't hear how many people, so I'm going to guess 2")

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



