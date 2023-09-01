#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from choosing_wait_position.final_lift_key_point.predict_pos import make_prediction
from narrow_space_navigation.waypoints import *
from tiago_controllers.controllers.controllers import Controllers
from sensor_msgs.msg import LaserScan
from PIL import Image
import numpy as np
from tiago_controllers.controllers.base_controller import CmdVelController
import random

MEAN_DISTANCE_THRESHOLD = 1.5
RANDOM_LIFT_JOKES = [
    "It seems like this button is feeling a bit neglected. ",
    "Would you mind giving the button some attention?",
    "I hate to be a bother, but this button seems to be feeling a bit left out. ",
    "Would you mind pressing it so it can shine like you?",
    "I know you're a shining star, but this button needs some love too.",
    "How about you give it a press and we'll both shine?",
    "This button may not be as bright as you, but it's still an important part of the team.",
    "Can you give it a press, please?",
    "Hey there, buddy! I hate to be pushy, but this button could use a little push from you. ",
    "I don't mean to push your buttons, but this one seems to be missing your touch.",
    "This button might not be the flashiest, but it's definitely feeling a bit blue.",
    "It's okay if you're not feeling as bright as this button, but can you still press for me?",
    "I know this button isn't as cool as you, but it still needs your help.",
    "Why don't elevators ever tell jokes? Because they're afraid of getting stuck in a long conversation!",
    "What do you call an elevator on a cruise ship? A stairway to heaven!",
    "Why did the elevator break up with the escalator? It just couldn't keep up with the ups and downs of the "
    "relationship!",
]


class CheckOpenDoor(smach.State):
    def __init__(self, controllers, voice):
        smach.State.__init__(self, outcomes=['success', 'failed'])

        self.controllers = controllers
        self.voice = voice
        self.cmd_vel = CmdVelController()

    def execute(self, userdata):
        in_lift = rospy.get_param("/in_lift/status")
        if in_lift:
            # face the door
            self.cmd_vel.rotate(60, 360/4, True)
            count = rospy.get_param("/counter_lift/counter")
            rospy.loginfo("count: " + str(count))
            if count > 3:
                return 'success'

            # set the param to count how many times it has failed in this state
            count += 1
            rospy.set_param("/counter_lift/counter", count)
        else:
            count = rospy.get_param("/counter_door/counter")
            rospy.loginfo("count: " + str(count))
            if count > 3:
                return 'success'

            # set the param to count how many times it has failed in this state
            count += 1
            rospy.set_param("/counter_door/counter", count)


        # tell lift jokes
        if rospy.get_param("/is_simulation"):
            rospy.loginfo("Waiting for the doors to open")
            rospy.loginfo(random.choice(RANDOM_LIFT_JOKES))
        else:
            self.voice.sync_tts("Just a quick update... I arrived at the lift. Waiting for the doors to open")
            self.voice.sync_tts(random.choice(RANDOM_LIFT_JOKES))
            rospy.sleep(2)

        # check for open door
        laser_scan = rospy.wait_for_message("/scan", LaserScan)
        filtered_ranges = laser_scan.ranges[len(laser_scan.ranges) // 3: 2 * len(laser_scan.ranges) // 3]
        mean_distance = np.nanmean(filtered_ranges)
        print('mean distance =====> ', mean_distance)
        if mean_distance < MEAN_DISTANCE_THRESHOLD or mean_distance == np.inf or mean_distance == np.nan:
            # if no, go back to waiting and a new joke
            print(mean_distance, 'is less than', MEAN_DISTANCE_THRESHOLD)
            return 'failed'
        else:
            return 'success'
