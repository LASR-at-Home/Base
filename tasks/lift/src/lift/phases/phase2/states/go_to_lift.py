#!/usr/bin/env python3
import os

import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from choosing_wait_position.final_lift_key_point.predict_pos import make_prediction, visualise_predictions
from narrow_space_navigation.waypoints import *
from tiago_controllers.controllers.controllers import Controllers
from PIL import Image, ImageOps, ImageFilter
from lift.defaults import PLOT_SAVE, PLOT_SHOW, TEST, DEBUG_PATH
import math

class GoToLift(smach.State):
    def __init__(self, controllers, voice):
        smach.State.__init__(self, outcomes=['success'])

        self.controllers = controllers
        self.voice = voice


    def rotate_keypoint(self,keypoint,degrees,image_o,image_r):

        width_o, height_o = image_o.size 
        image_origin = [width_o/2,height_o/2]

    

        width_r , height_r = image_r.size

        x_offset, y_offset = (width_r - width_o)/2 , (height_r-height_o)/2
      #  print("image origin: ", image_origin)
       # print("keypoint in rotate", keypoint)
        x, y, bin = keypoint[0]
        ox, oy = image_origin

        tx = x - ox 
        ty = y - oy 
        #print("new tx,ty ", (tx,ty))


        x_rotate = tx*math.cos(math.radians(degrees)) - ty*math.sin(math.radians(degrees))
        y_rotate = ty*math.cos(math.radians(degrees)) + tx*math.sin(math.radians(degrees))

      #  print('x_rotate,y_rotate', (x_rotate,y_rotate))

        x = x_rotate + 480
        y = y_rotate + 640
        
        keypoint_rotated = [[x+x_offset,y+y_offset,1]]
        return keypoint_rotated

    def flip_keypoint(self, keypoint,image):
        width, height = image.size
        width = 630
        height = 480
            #to flip keypoint back: (x,y) -> (x,length-y-1)
        #print("keypoint in flip: ", keypoint)
        keypoint = keypoint[0]
        keypoint_flipped = [[keypoint[0],(height-keypoint[1]-1)],1]
        return keypoint_flipped

    def mirror_keypoint(self,keypoint,image):
        width, height = image.size
        keypoint = keypoint[0]

        keypoint_flipped = [[(width-keypoint[0]-1),keypoint[1],1]]
        return keypoint_flipped

    def execute(self, userdata):
        # position robot in front of the lift waiting area

        # self.controllers.base_controller.sync_to_pose(get_pose_from_param('/wait/pose'))

        # do zoe's stuff
        w = Waypoint()
        print("getting lift information  {} ".format(rospy.get_param("/is_simulation")))
        warped, analysis, M = w.get_lift_information(is_lift=False, is_sim=rospy.get_param("/is_simulation"))
        image = Image.fromarray(warped)
        #Make the white space bigger to make it harder for the network: 
        image = image.filter(ImageFIlter.MinFilter(3))

        print("Debug Path: ", DEBUG_PATH)
        
        print(os.getcwd())
        if PLOT_SAVE:
            image.save(DEBUG_PATH + "/zoe_predict_pos_test_before_rotate" + str(TEST) + ".jpg")
        
        degrees = 90
        #Rotate the image to make sure it's facing the door properly: 
        image_rotated = image.rotate(degrees)
        image_rotated = ImageOps.mirror(image_rotated)

        if PLOT_SAVE:
            image_rotated.save(DEBUG_PATH + "/zoe_predict_pos_test_after_rotate" + str(TEST) + ".jpg")


        #predicting with rotated image: 
        
        image_path = DEBUG_PATH + "/zoe_predict_pos_test_after_rotate" + str(TEST) + ".jpg"
        #print("image_path: ", image_path)
        bbox, keypoint = make_prediction(image_path)
        if keypoint is None and bbox is None:
            keypoint = analysis[3]

        keypoint = self.mirror_keypoint(keypoint,image_rotated)
        keypoint = self.rotate_keypoint(keypoint,(360-degrees),image_rotated,image)
    #        print(f"keypoints predicted after rotate: {keypoint}")

        
        #print("keypoint flipped: ", keypoint_flipped)
        visualise_predictions(warped, bbox, keypoint)
     #   print(type(keypoint))
        

        global_points = w.local_to_global_points(M=M, points=keypoint, is_lift=True)
        p = Pose()
        p.position.x = global_points[0][0]
        p.position.y = global_points[0][1]
       # p.position.z = -0.5923
       # p.orientation.w = 0.8056
        result = self.controllers.base_controller.sync_to_pose(p)
        print(f"result: {result}")
        if not result:
            # if it fails go to predetermined place
            result = self.controllers.base_controller.sync_to_pose(get_pose_from_param('/wait_centre/pose'))

        return 'success'