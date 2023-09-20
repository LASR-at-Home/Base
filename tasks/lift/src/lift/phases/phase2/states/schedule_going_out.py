#!/usr/bin/env python3
import numpy as np
import smach
import rospy
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from std_msgs.msg import Empty
from tiago_controllers.helpers.nav_map_helpers import clear_costmap
from interaction_module.srv import AudioAndTextInteraction, AudioAndTextInteractionRequest, \
    AudioAndTextInteractionResponse
from lift.defaults import TEST, PLOT_SHOW, PLOT_SAVE, DEBUG_PATH, DEBUG, RASA
from tiago_controllers.helpers.nav_map_helpers import is_close_to_object, rank
import json
from lasr_object_detection_yolo.detect_objects_v8 import detect_objects, perform_detection, debug
from sensor_msgs.msg import PointCloud2


class ScheduleGoingOut(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.default = default
        self.head_motions = {
            'look_straight': self.default.controllers.head_controller.look_straight,
            'look_right': self.default.controllers.head_controller.look_right,
            'look_left': self.default.controllers.head_controller.look_left
        }

    def listen(self):
        resp = self.default.speech()
        if not resp.success:
            self.default.voice.speak("Sorry, I didn't get that")
            return self.listen()
        resp = json.loads(resp.json_response)
        rospy.loginfo(resp)
        return resp

    def hear_wait(self):
        resp = self.listen()

        if resp["intent"]["name"] == "negotiate_lift":
            # I'm going to wait
            wait = resp["entities"].get("wait_command", [])
            if not wait:
                self.default.voice.speak("Sorry, did you say wait? I didn't understand.")
                return self.hear_wait()
            else:
                return True
        else:

            return False

    def affirm(self):
        resp = self.listen()
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


    def is_anyone_in_front_of_me(self):
        detections = 0
        for motion in self.head_motions:
            self.head_motions[motion]()
            rospy.sleep(1)
            pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
            polygon = rospy.get_param('/corners_lift')
            detections, im = perform_detection(self.default, pcl_msg, polygon, ['person'])
            if len(detections) > 0:
                self.default.controllers.head_controller.look_straight()
                return True
        return False


    def execute(self, userdata):
        is_closer_to_door = not self.is_anyone_in_front_of_me()

        if is_closer_to_door:
            self.default.voice.speak("I am the closest to the door so I have to exit first")
            # clear costmap
            clear_costmap()
            # go to centre waiting area
            state = self.default.controllers.base_controller.ensure_sync_to_pose(get_pose_from_param('/wait_in_front_lift_centre/pose'))
            if not state:
                return 'failed'
            # turn around
            self.default.voice.speak("Just minding my own business!")
            self.default.controllers.base_controller.rotate_to_face_object(object_name='/door/pose')
            # wait for the person to exit
            rospy.sleep(2)
            self.default.voice.speak("Should I wait more for you?")

            hear_wait = "yes"
            count = 0
            while hear_wait and count < 5:
                if RASA:
                    hear_wait = self.affirm()
                    if hear_wait == "yes":
                        self.default.voice.speak("I will wait more!")
                        rospy.sleep(5)
                    else:
                        self.default.voice.speak("I think I've finished waiting!")
                        break
                    count += 1

            return 'success'
        else:
            self.default.voice.speak("I am not the closest to the door.")
            self.default.voice.speak("I will wait inside the lift because this is not my floor")
            rospy.set_param("/from_schedule", True)
            return 'success'
