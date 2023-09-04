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
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion, Point
import random

import rospy
import math
from geometry_msgs.msg import Twist
from tf_module.transformations import euler_from_quaternion, quaternion_from_euler
# from tf_module.tf_transforms import tranform_pose

door_open = False
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
# Constants for door detection (you can adjust these)
DOOR_WIDTH_MIN = 0.05  # Minimum door width in meters
DOOR_WIDTH_MAX = 0.07  # Maximum door width in meters
DISTANCE_THRESHOLD = 1.5  # Maximum distance to consider an opening as a potential door
EXPECTED_DOOR_X = 4.21764755249
EXPECTED_DOOR_Y = -4.48686313629
EXPECTED_DOOR_ANGLE_DEG = 90.0


class CheckOpenDoor(smach.State):
    def __init__(self, controllers, voice):
        # smach.State.__init__(self, outcomes=['success'])
        smach.State.__init__(self, outcomes=['success', 'failed'])

        self.controllers = controllers
        self.voice = voice
        self.cmd_vel = CmdVelController()

    def get_current_robot_pose(self):
        robot_pose = rospy.wait_for_message("/robot_pose", PoseWithCovarianceStamped)
        return robot_pose.pose.pose


    # maybe move to base_controller
    def calculate_angle(self, robot_pose, door_position):
        delta_x = door_position.position.x - robot_pose.position.x
        delta_y = door_position.position.y - robot_pose.position.y
        angle_to_door = math.atan2(delta_y, delta_x)

        if angle_to_door > math.pi / 2:
            angle_to_door -= math.pi
        elif angle_to_door < -math.pi / 2:
            angle_to_door += math.pi

        (x, y, z, w) = quaternion_from_euler(0, 0, angle_to_door)
        quaternion = Quaternion(x, y, z, w)
        pose = Pose(position=Point(robot_pose.position.x, robot_pose.position.y, 0.0), orientation=quaternion)
        return pose


    def rotate_to_face_door(self):
        robot_pose = self.get_current_robot_pose()
        door_position = get_pose_from_param("/door/pose")

        rotation_angle = self.calculate_angle(robot_pose, door_position)

        self.controllers.base_controller.sync_to_pose(rotation_angle)

    def rotate_to_face_door_new(self):
        """
        Rotate to face the door jareds
        """
        door_position = get_pose_from_param("/door/pose")
        rotation_angle = self.controllers.base_controller.compute_face_quat(door_position.position.x, door_position.position.y)
        self.controllers.base_controller.sync_to_pose(rotation_angle)

    def counter(self, topic="/counter_lift/counter"):
        count = rospy.get_param(topic)
        rospy.loginfo("count: " + str(topic) + "---> " + str(count))
        if count > 3:
            return 'success'

        # set the param to count how many times it has failed in this state
        count += 1
        rospy.set_param(topic, count)


    # def preprocess_laser_data(self, ranges):
    #     window_size = 5
    #     smoothed_ranges = np.convolve(ranges, np.ones(window_size) / window_size, mode='same')
    #     return smoothed_ranges
    #
    # def segment_laser_data(self, ranges):
    #     openings = []
    #     current_opening = None
    #     for i, range_measurement in enumerate(ranges):
    #         if range_measurement > DISTANCE_THRESHOLD:
    #             if current_opening is not None:
    #                 current_opening['end_index'] = i
    #                 openings.append(current_opening)
    #                 current_opening = None
    #         else:
    #             if current_opening is None:
    #                 current_opening = {'start_index': i, 'end_index': i}
    #             else:
    #                 current_opening['end_index'] = i
    #
    #     return openings

    # def is_door(self, msg, ranges, opening, current_time):
    #     global door_open
    #
    #     width = (opening['end_index'] - opening['start_index']) * msg.angle_increment * msg.range_max
    #
    #     if DOOR_WIDTH_MIN <= width <= DOOR_WIDTH_MAX:
    #         distance = (ranges[opening['start_index']] + ranges[opening['end_index']]) / 2.0
    #
    #         if distance <= DISTANCE_THRESHOLD:
    #             opening_angle = (opening['start_index'] + opening[
    #                 'end_index']) / 2.0 * msg.angle_increment + msg.angle_min
    #
    #             expected_door_angle = np.radians(EXPECTED_DOOR_ANGLE_DEG)
    #
    #             expected_door_pose = PoseStamped()
    #             expected_door_pose.header.stamp = current_time
    #             expected_door_pose.header.frame_id = msg.header.frame_id
    #             expected_door_pose.pose.position.x = EXPECTED_DOOR_X
    #             expected_door_pose.pose.position.y = EXPECTED_DOOR_Y
    #             expected_door_pose.pose.orientation.x, expected_door_pose.pose.orientation.y, expected_door_pose.pose.orientation.z, expected_door_pose.pose.orientation.w = euler_from_quaternion(
    #                 [0, 0, expected_door_angle])
    #
    #
    #             expected_door_pose_transformed = tranform_pose(expected_door_pose, msg.header.frame_id, current_time)
    #
    #             # Calculate the distance and angle between the detected opening and the expected door pose
    #             distance_to_expected_door = np.sqrt(
    #                 (expected_door_pose_transformed.pose.position.x - msg.pose.pose.position.x) ** 2 + (
    #                             expected_door_pose_transformed.pose.position.y - msg.pose.pose.position.y) ** 2)
    #             angle_to_expected_door = np.arctan2(
    #                 expected_door_pose_transformed.pose.position.y - msg.pose.pose.position.y,
    #                 expected_door_pose_transformed.pose.position.x - msg.pose.pose.position.x)
    #
    #             # Check if the detected opening is close to the expected door location and angle
    #             if distance_to_expected_door < DISTANCE_THRESHOLD and abs(angle_to_expected_door) < np.radians(
    #                     15):  # Adjust angle threshold as needed
    #                 return True
    #
    #     return False
    #
    # def laser_callback(self, msg):
    #     global door_open  # Access the global door_open variable
    #     # Extract the laser scan data
    #     ranges = msg.ranges  # List of range measurements
    #
    #     # Preprocess the LaserScan data (smoothing)
    #     smoothed_ranges = self.preprocess_laser_data(ranges)
    #
    #     # Perform segmentation to detect openings (potential doors)
    #     openings = self.segment_laser_data(smoothed_ranges)
    #
    #     # Analyze the detected openings
    #     for opening in openings:
    #         if self.is_door(msg, ranges, opening):
    #             rospy.loginfo("Potential door detected at distance {:.2f} meters with width {:.2f} meters".format(
    #                 opening['distance'], opening['width']))
    #             return True
    #
    #     return False

    # to add in execute
     # laser_scan = rospy.wait_for_message("/scan", LaserScan)
        # sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        # while not rospy.is_shutdown():
        #     # Access the door_open variable to check if a door is open
        #     if door_open:
        #         rospy.loginfo("The door is open!")
        #         return 'success'
        #     else:
        #         rospy.loginfo("The door is closed!")


    def execute(self, userdata):
        in_lift = rospy.get_param("/in_lift/status")
        if in_lift:
            # face the door
            # self.rotate_to_face_door()
            self.rotate_to_face_door_new()
            self.counter(topic="/counter_lift/counter")
            self.voice.speak("Just a quick update... I am checking if the doors are open.")
        else:
            self.counter(topic="/counter_door/counter")
            self.voice.speak("Just a quick update... I arrived at the lift. Waiting for the doors to open")

        # tell a joke
        self.voice.speak(random.choice(RANDOM_LIFT_JOKES))
        rospy.sleep(2)

        self.voice.speak("Rotating to face the door")
        self.rotate_to_face_door_new()
        self.voice.speak("Checking the door now")

        # check for open door
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


