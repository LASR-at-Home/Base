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
from lift.defaults import PLOT_SAVE, PLOT_SHOW, TEST, DEBUG_PATH, DEBUG

class GoToLift(smach.State):
    def __init__(self, controllers, voice):
        smach.State.__init__(self, outcomes=['success'])

        self.controllers = controllers
        self.voice = voice

    def execute(self, userdata):
        # position robot in front of the lift waiting area - ZOE
        w = Waypoint()

        # get the lift info
        warped, analysis, M = w.get_lift_information(is_lift=False, is_sim=rospy.get_param("/is_simulation"))

        # save to the NN format
        image = Image.fromarray(warped)

        if PLOT_SAVE:
            image.save(DEBUG_PATH + "/zoe_predict_pos_test" + str(TEST) + ".jpg")
        if DEBUG > 3:
            print(os.getcwd())

        # make the prediction with NN
        image_path = DEBUG_PATH + "/zoe_predict_pos_test" + str(TEST) + ".jpg"
        bbox, keypoint = make_prediction(image_path)
        if keypoint is None and bbox is None:
            keypoint = analysis[3]

        if DEBUG > 3:
            if DEBUG > 4:
                visualise_predictions(warped, bbox, keypoint)
            print(f"keypoint: {keypoint}")
            print(type(keypoint))

        # transform to the global frame
        global_points = w.local_to_global_points(M=M, points=keypoint, is_lift=True)
        p = Pose()
        p.position.x = global_points[0][0]
        p.position.y = global_points[0][1]

        # TODO: maybe don't hardcode this, take the orientation from the door
        p.position.z = -0.5923
        p.orientation.w = 0.8056

        state = self.controllers.base_controller.sync_to_pose(p)

        if DEBUG > 3:
            print(f" The sync to pose res in go to lift: {state}")

        # ensure sync to pose
        if not state:
            # if it fails go to predetermined place
            state = self.controllers.base_controller.ensure_sync_to_pose(get_pose_from_param('/wait_centre/pose'))
            rospy.loginfo("The sync to pose res in go to lift wait centre is {}".format(state))

        return 'success'