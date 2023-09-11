#!/usr/bin/env python3

import smach, rospy
import numpy as np
import random
from sensor_msgs.msg import LaserScan
from tiago_controllers.controllers.base_controller import CmdVelController
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from tiago_controllers.helpers.nav_map_helpers import counter
from narrow_space_navigation.waypoints import *
import rospy
# from tf_module.tf_transforms import tranform_pose

door_open = False
MEAN_DISTANCE_THRESHOLD = 0.5
RANDOM_LIFT_JOKES = [
    "Why don't elevators ever tell jokes? Because they're afraid of getting stuck in a long conversation!",
    "What do you call an elevator on a cruise ship? A stairway to heaven!",
    "Why did the elevator break up with the escalator? It just couldn't keep up with the ups and downs of the "
    "relationship!",
    "It seems like this button is feeling a bit neglected. ",
    "Would you mind giving the button some attention?",
    "Why did the elevator break up with the escalator? It just couldn't keep up the relationship!",
    "Do you know why elevators are so good at telling jokes? Because they have great 'uplifting' humor!",
    "Elevators are great at keeping secrets. They have a lot of 'uplifting' experiences to share!",
    "What's an elevator's favorite game? Up and down!",
    "Elevators always make 'uplifting' conversation. They never bring you down!",
    "Why did the elevator get in trouble at school? It couldn't keep its hands to itself!",
    "Elevators are like good friends; they're always there for an 'uplifting' chat!",
    "What do you call an elevator that's not working? A 'downer-vator'!",
    "Elevators are like magic rooms; they can lift your spirits in seconds!",
    "Why don't elevators ever get tired? Because they have plenty of 'up' time!",
    "Elevators are like life: they have their ups and downs, but they always get you where you need to go!",
    "What do you call a group of people waiting for an elevator? A 'lift' club!",
    "Elevators are great at parties; they know how to 'elevate' the mood!",
    "Why did the smartphone take the elevator to work? It wanted to avoid the stairs and 'elevate' its status!",
    "Elevators have one job, and they do it 'upliftingly' well!",
    "If you ever feel down, take the elevator. It'll give you a lift!",
    "Elevators are the unsung heroes of modern life. They're always there to 'lift' you up!",
    "What do you call an elevator with an attitude? An 'el-evader'!",
    "Elevators are like the quiet kid in class; they always have something 'uplifting' to say!",
    "Why did the elevator go to therapy? It had too many 'upsetting' experiences!",
    "Elevators are the best travel companions; they'll always 'uplift' your journey!",
    "What's an elevator's favorite type of music? Elevator music, of course!",
    "Elevators are like libraries; they have a lot of 'uplifting' stories to tell!",
    "Why did the elevator join the gym? It wanted to get 'lift' and fit!",
    "Elevators are like superheroes; they save you from the 'downs' of climbing stairs!",
    "What's an elevator's favorite button? The 'up' button, of course!",
]

used_jokes = []
from std_msgs.msg import Bool
door_detected = False

class CheckOpenDoor(smach.State):
    def __init__(self, controllers, voice):
        # smach.State.__init__(self, outcomes=['success'])
        smach.State.__init__(self, outcomes=['success', 'failed'])

        self.controllers = controllers

        self.sub = rospy.Subscriber('/door_detected', Bool, self.door_detected_callback)
        print("door detected {}".format(door_detected))
        self.voice = voice
        self.cmd_vel = CmdVelController()

    def get_joke(self):
        if not used_jokes:
            used_jokes.extend(RANDOM_LIFT_JOKES)
            random.shuffle(used_jokes)
        return used_jokes.pop()

    def door_detected_callback(self, msg):
        global door_detected
        door_detected = msg.data
        
    def execute(self, userdata):

        in_lift = rospy.get_param("/in_lift/status")

        # ensure the head is straight
        self.controllers.head_controller.look_straight()

        # rotate to face the door
        self.voice.speak("Rotating to face the door")
        self.controllers.base_controller.rotate_to_face_object(object_name='/door/pose')

        topic = "/counter_lift/counter" if in_lift else "/counter_door/counter"
        message = "I am in the lift. Waiting for the doors to open" if in_lift else "I arrived at the door. Waiting for the doors to open"
        res = counter(topic=topic)

        if res == "counter":
            return 'success'
        self.voice.speak(message)

        self.controllers.base_controller.rotate_to_face_object(object_name='/door/pose')

        # tell a joke
        self.voice.speak("I will tell you a joke in the meantime.")
        self.voice.speak(self.get_joke())
        rospy.sleep(1)

        self.voice.speak("Now checking the door")


        # check for open door
        while door_detected:
            rospy.logerr("the door detected {}".format(door_detected))
            self.voice.speak("I am still waiting for the door to open")
            rospy.sleep(1)
            if door_detected:
                self.voice.speak("The door is open. I will give the way to the humans now, because I am a good robot.")
                return 'success'


        if door_detected:
            return 'failed'
        else:
            return 'success'



        laser_scan = rospy.wait_for_message("/scan", LaserScan)
        filtered_ranges = laser_scan.ranges[len(laser_scan.ranges) // 3: 2 * len(laser_scan.ranges) // 3]
        mean_distance = np.nanmean(filtered_ranges)
        rospy.loginfo('mean distance =====> {}'.format(mean_distance))
        if mean_distance < MEAN_DISTANCE_THRESHOLD or mean_distance == np.inf or mean_distance == np.nan:
            # if no, go back to waiting and a new joke
            rospy.loginfo(" the mean distance = {} is less than thres = {}".format(mean_distance, MEAN_DISTANCE_THRESHOLD))
            return 'failed'
        else:
            self.voice.speak("Oh its open! I will give the way to the humans now, because I am a good robot.")
            return 'success'


