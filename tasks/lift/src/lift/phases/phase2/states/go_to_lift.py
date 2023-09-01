#!/usr/bin/env python3
import os

import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from choosing_wait_position.final_lift_key_point.predict_pos import make_prediction
from narrow_space_navigation.waypoints import *
from tiago_controllers.controllers.controllers import Controllers
from PIL import Image

class GoToLift(smach.State):
    def __init__(self, controllers, voice):
        smach.State.__init__(self, outcomes=['success'])

        self.controllers = controllers
        self.voice = voice

    def execute(self, userdata):
        # position robot in front of the lift waiting area

        # do zoe's stuff
        w = Waypoint()
        warped, analysis, M = w.get_lift_information(is_lift=False)
        image = Image.fromarray(warped)
        print(os.getcwd())
        image.save("test.jpg")
        image_path = "test.jpg"
        bbox, keypoint = make_prediction(image_path)
        if keypoint is None and bbox is None:
            keypoint = analysis[3]

        print(f"keypoint: {keypoint}")
        print(type(keypoint))
        global_points = w.local_to_global_points(M=M, points=keypoint, is_lift=True)
        p = Pose()
        p.position.x = global_points[0][0]
        p.position.y = global_points[0][1]
        p.orientation.w = 1
        result = self.controllers.base_controller.sync_to_pose(p)

        # if it fails go to predetermined place
        result = self.controllers.base_controller.sync_to_pose(get_pose_from_param('/wait_centre'))

        return 'success'