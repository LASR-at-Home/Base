#!/usr/bin/env python3

import rospy
from tiago_controllers.controllers.base_controller import BaseController, CmdVelController
from tiago_controllers.controllers.head_controller import HeadController
from sensor_msgs.msg import Image
from lasr_object_detection_yolo.detect_objects_v8 import detect_objects
import numpy as np
from math import atan


# TODO: REFACTOR!!!!

MOVING_INCR_X = 0.1
MOVING_INCR_Y = 0.1
DISTANCE = 2000
PIXEL_DIST_CONST_MM = 0.2645833333


class LookAt:

    def __init__(self, head_controller: HeadController, base_controller: BaseController, cmd_vel_controller, object: str, model="coco"):
        self._base_controller = base_controller
        self._head_controller = head_controller
        self._cmd_vel_controller = cmd_vel_controller
        self._object = object
        self._model = model

    def look_at(self, xywh, head_rot):
        self.turn_to(xywh, head_rot)
        self._head_controller.sync_reach_to(0,0)
        self.aim_head()

    def turn_to(self, xywh, head_rot):
        x = xywh[0]-(xywh[3]/2)
        is_clockwise = True
        if x + (-100*head_rot) < 0:
            print("Counter clockwise ------------------")
            is_clockwise = False
        angle = atan(DISTANCE/(PIXEL_DIST_CONST_MM*abs(x)))
        print("ANGLE: ", angle)
        self._cmd_vel_controller.rotate(70, angle, is_clockwise)

    def aim_head(self):
        robot_pos = np.array(self._base_controller.get_current_pose()[:2])
        img_sensor_msg = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        hum_centroid = [img_sensor_msg.height // 2, img_sensor_msg.width // 2]
        hum_centroid[0], hum_centroid[1], xywh = self.__human_detected(robot_pos)
        tmp_hum_x, tmp_hum_y, tmp_xywh = self.__human_detected(robot_pos)
        if tmp_hum_x and tmp_hum_y:
            rospy.loginfo("There is small difference between the centroids")
            hum_centroid[0] = tmp_hum_x
            hum_centroid[1] = tmp_hum_y
            print(f"moving head and the centroids")
            self.__move_robot_head(hum_centroid[0], hum_centroid[1], tmp_xywh, robot_pos)

    def __move_robot_head(self, x, y, _xywh,  robot_pos):
        try:
            hum_x, hum_y, xywh = x,y, _xywh
            img_sensor_msg = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
            rob_h, rob_w = (img_sensor_msg.height // 2), (img_sensor_msg.width // 2)
            print("-"*40)
            print(f"the robots pos is h = {rob_h}, and the w = {rob_w}")
            print(f"the human pos is h = {xywh[3]}, and the w = {xywh[2]}, and all of xywh = {xywh}")
            i = 0
            moving_incr_x = 0.0
            moving_incr_y = 0.0

            while abs(xywh[0] + (xywh[2]//2) - rob_w) > 50 and abs(xywh[1] + xywh[3] - rob_h) > 25:
                print(f"in the while and {abs(xywh[0] + (xywh[2]//2) - rob_w)} and the y axis {abs(xywh[1] + xywh[2] - rob_h)} is bigger than 25")
                moving_incr_x, moving_incr_y = self.check_moving_side(
                    xywh[0] + (xywh[2]//2),
                    xywh[1] + (xywh[3]//2),
                    rob_w,
                    rob_h,
                    moving_incr_x,
                    moving_incr_y
                )
                if moving_incr_x is None:
                    return
                self._head_controller.sync_reach_to(moving_incr_x, moving_incr_y)
                hum_x, hum_y, xywh = self.__human_detected(robot_pos)
                print("the new human centroid x is = ", xywh)
                i += 1

            print(f"I finished moving my head to gaze at you")
        except:
            rospy.logwarn("I cound NOT reach that head position")

    def check_moving_side(self, hum_x, hum_y, rob_x, rob_y, _moving_incr_x, _moving_incr_y):
        try:
            if abs(_moving_incr_x) < 1.20 and 0.70 > _moving_incr_y > -0.9:
                if rob_x > hum_x:
                    moving_incr_x = _moving_incr_x + MOVING_INCR_X
                else:
                    moving_incr_x = _moving_incr_x - MOVING_INCR_X
                if rob_y > hum_y:
                    moving_incr_y = _moving_incr_y - MOVING_INCR_Y
                else:
                    moving_incr_y = _moving_incr_y + MOVING_INCR_Y
            return moving_incr_x, moving_incr_y
        except:
            rospy.logwarn(" Cannot move the head ")
            return None, None
            # return _moving_incr_x, _moving_incr_y

    def __human_detected(self, robot_pos):
        detections = detect_objects(["person"])
        if detections:
            first = detections[0]
            return first.xywh[0], first.xywh[1], first.xywh
        rospy.loginfo("I can't see any people")
        return None, None, None
