#!/usr/bin/env python3
import os

import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from choosing_wait_position.final_lift_key_point.predict_pos import make_prediction, visualise_predictions
from narrow_space_navigation.waypoints import *
from tiago_controllers.controllers.controllers import Controllers
from PIL import Image
from lift.defaults import PLOT_SAVE, PLOT_SHOW, TEST, DEBUG_PATH

class GoToLift(smach.State):
    def __init__(self, controllers, voice):
        smach.State.__init__(self, outcomes=['success'])

        self.controllers = controllers
        self.voice = voice

    def execute(self, userdata):
        # position robot in front of the lift waiting area

        # self.controllers.base_controller.sync_to_pose(get_pose_from_param('/wait/pose'))

        # do zoe's stuff
        w = Waypoint()
        print("getting lift information  {} ".format(rospy.get_param("/is_simulation")))
        warped, analysis, M = w.get_lift_information(is_lift=False, is_sim=rospy.get_param("/is_simulation"))
        image = Image.fromarray(warped)

        print("Debug Path: ", DEBUG_PATH)
        
        print(os.getcwd())
        if PLOT_SAVE:
            image.save(DEBUG_PATH + "/zoe_predict_pos_test_before_rotate" + str(TEST) + ".jpg")

        #image = image.rotate(180)

        if PLOT_SAVE:
            image.save(DEBUG_PATH + "/zoe_predict_pos_test_after_rotate" + str(TEST) + ".jpg")

        image_path = DEBUG_PATH + "/zoe_predict_pos_test_after_rotate" + str(TEST) + ".jpg"
        print("image_path: ", image_path)
        bbox, keypoint = make_prediction(image_path)
        visualise_predictions(warped, bbox, keypoint)
        if keypoint is None and bbox is None:
            keypoint = analysis[3]

        print(f"keypoint: {keypoint}")
        print(type(keypoint))
        global_points = w.local_to_global_points(M=M, points=keypoint, is_lift=True)
        p = Pose()
        p.position.x = global_points[0][0]
        p.position.y = global_points[0][1]
        p.position.z = -0.5923
        p.orientation.w = 0.8056
        result = self.controllers.base_controller.sync_to_pose(p)
        print(f"result: {result}")
        if not result:
            # if it fails go to predetermined place
            result = self.controllers.base_controller.sync_to_pose(get_pose_from_param('/wait_centre/pose'))

        return 'success'