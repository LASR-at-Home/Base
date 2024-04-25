#!/usr/bin/env python3
import os
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from choosing_wait_position.final_lift_key_point.predict_pos import (
    make_prediction,
    visualise_predictions,
)
from narrow_space_navigation.waypoints import *
from tiago_controllers.controllers.controllers import Controllers
from PIL import Image
from lift.defaults import PLOT_SAVE, PLOT_SHOW, TEST, DEBUG_PATH, DEBUG
from PIL import Image, ImageOps, ImageFilter
import math


class GoToLift(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=["success"])
        self.default = default

        # self.controllers = controllers
        # self.voice = voice

    def rotate_keypoint(self, keypoint, degrees, image_o, image_r):

        width_o, height_o = image_o.size
        image_origin = [width_o / 2, height_o / 2]
        width_r, height_r = image_r.size

        x_offset, y_offset = (width_r - width_o) / 2, (height_r - height_o) / 2
        x, y, bin = keypoint[0]
        ox, oy = image_origin

        tx = x - ox
        ty = y - oy

        x_rotate = tx * math.cos(math.radians(degrees)) - ty * math.sin(
            math.radians(degrees)
        )
        y_rotate = ty * math.cos(math.radians(degrees)) + tx * math.sin(
            math.radians(degrees)
        )

        # x = x_rotate + 480
        # y = y_rotate + 640

        x = x_rotate + ox
        y = y_rotate + oy

        keypoint_rotated = [[x + x_offset, y + y_offset, 1]]
        return keypoint_rotated

    def flip_keypoint(self, keypoint, image):
        width, height = image.size
        # to flip keypoint back: (x,y) -> (x,length-y-1)
        keypoint = keypoint[0]
        keypoint_flipped = [[keypoint[0], (height - keypoint[1] - 1)], 1]
        return keypoint_flipped

    def mirror_keypoint(self, keypoint, image):
        width, height = image.size
        keypoint = keypoint[0]

        keypoint_flipped = [[(width - keypoint[0] - 1), keypoint[1], 1]]
        return keypoint_flipped

    def execute(self, userdata):
        # position robot in front of the lift waiting area - ZOE
        w = Waypoint()

        # get the lift info
        warped, analysis, M = w.get_lift_information(
            is_lift=False, is_sim=rospy.get_param("/is_simulation")
        )

        # save to the NN format
        image = Image.fromarray(warped)
        # make the prediction with NN
        image_path = DEBUG_PATH + "/zoe_predict_pos_test" + str(TEST) + ".jpg"
        # Make the white space bigger to make it harder for the network:
        image = image.filter(ImageFilter.MinFilter(3))

        if DEBUG > 3:
            print("Debug Path: ", DEBUG_PATH)
        if PLOT_SAVE:
            print(os.getcwd())
            image.save(DEBUG_PATH + "/zoe_predict_pos_test" + str(TEST) + ".jpg")
            # image.save(DEBUG_PATH + "/zoe_predict_pos_test_before_rotate" + str(TEST) + ".jpg")

        bbox, keypoint = make_prediction(image_path)
        if keypoint is None and bbox is None:
            keypoint = analysis[3]

        if DEBUG > 3:
            if DEBUG > 4:
                visualise_predictions(warped, bbox, keypoint)
            print(f"keypoint: {keypoint}")
            print(type(keypoint))

        # self.default.voice.speak("Let me talk to the audience.")

        # transform to the global frame
        global_points = w.local_to_global_points(M=M, points=keypoint, is_lift=True)
        p = Pose()
        p.position.x = global_points[0][0]
        p.position.y = global_points[0][1]

        # TODO: maybe don't hardcode this, take the orientation from the door
        p.position.z = -0.5923
        p.orientation.w = 0.8056

        state = self.default.controllers.base_controller.sync_to_pose(p)

        if DEBUG > 3:
            print(f" The sync to pose res in go to lift: {state}")

        # ensure sync to pose
        if not state:
            state = self.default.controllers.base_controller.ensure_sync_to_pose(
                get_pose_from_param("/wait_centre/pose")
            )
            rospy.loginfo(
                "The sync to pose res in go to lift wait centre is {}".format(state)
            )

        return "success"
