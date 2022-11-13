#!/usr/bin/env python3

import numpy as np
import rospy
from cv_bridge3 import CvBridge, cv2
from sensor_msgs.msg import Image, PointCloud2
import smach
from point_to_person import bb_to_centroid
from yolo_object_detection.srv import YoloDetection

from open_pose.srv import DetectKeypoints
from dialogflow_speech.utils import talk

import json

PEOPLE_PER_SOFA = 2

class DetectSeatablePositionState(smach.State):
    def __init__(self, head_controller):
        smach.State.__init__(self, outcomes=['got_available_seat', 'no_available_seat'], output_keys=['seatable_pose'])
        
        self.head_controller = head_controller

        if self.head_controller:
            self.rotate_head = self.head_controller.sync_reach_to
        else:
            self.rotate_head = lambda *args, **kwargs : None


        rospy.wait_for_service("yolo_detection")
        self.detect = rospy.ServiceProxy("yolo_detection", YoloDetection)

        rospy.wait_for_service("detect_keypoints")
        self.detect_keypoints = rospy.ServiceProxy("detect_keypoints", DetectKeypoints)

        self.bridge = CvBridge()

    '''Returns the overlap percentage'''
    def get_percentage_overlap(self, xywh1, xywh2):
        area_chair = xywh1[2] * xywh1[3]
        overlap_area = self.overlap_area(self.get_corners(*xywh1), self.get_corners(*xywh2))
        return overlap_area/area_chair*100

    '''returns the overlapped area between two bounding boxes'''
    def overlap_area(self, A, B):
        left = max(A['x1'], B['x1'])
        bottom = max(A['y2'], B['y2'])
        right = min(A['x2'], B['x2'])
        top = min(A['y1'], B['y1'])
        return (right-left)*(top-bottom)

    '''checks if two bounding boxes overlap'''
    def is_overlapping(self, A, B):
        return (A['x1'] < B['x2'] and A['x2'] > B['x1'] and
                A['y1'] > B['y2'] and A['y2'] < B['y1'])

    '''
    returns the top right and bottom left corner
   (x1, y1) X______________
            |             |
            |_____________X  (x2, y2)
    '''
    def get_corners(self, x, y, w, h):
        return {
            'x1': x,
            'y1': y + h,
            'x2': x + w,
            'y2': y
        }

    ''' 
    Get available bbox [x, y, w, h] of sofa, 
    taking into account that the sofa can seat 2 people available
    '''
    def get_bbox_available_space_sofa(self, S, P):
        left = P['x1'] - S['x1']
        right = S['x2'] - P['x2']
        if left > right:  # left-hand side of couch is available
            return S['x1'], S['y2'], left, S['y1']-S['y2']
        else:  # right-hand side of couch is available
            return P['x1'], S['y2'], right, S['y1']-S['y2']

    def bb_intersects(self, xywh1, xywh2):
        c1 = self.get_corners(*xywh1)
        c2 = self.get_corners(*xywh2)

        return self.is_overlapping(c1, c2)

    def execute(self, userdata):
        talk("I'm looking for somewhere for you to sit.")
        joint_1_direction = 0.4
        joint_2_direction = 0.0
        
        seatables = []
        people = []

        for _ in range(4):
            joint_1_direction *= -1
            joint_2_direction -= 0.1
            self.rotate_head(joint_1_direction, joint_2_direction, velocities=[0.1, 0.0])

            pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
            img_msg = rospy.wait_for_message("/xtion/rgb/image_raw", Image)

            im = self.bridge.imgmsg_to_cv2_np(img_msg)

            detections = self.detect(img_msg, pcl_msg, "coco", 0.5, 0.3).detected_objects
            seatables.extend([d for d in detections if d.name in ["sofa", "chair"]])
            people.extend([d for d in detections if d.name == "person"])


        for seatable in seatables:

            if seatable.name == "chair":

                for person in people:

                    if not (self.bb_intersects(seatable.xywh, person.xywh) and self.get_percentage_overlap(seatable.xywh, person.xywh) > 0.7):
                        userdata.seatable_pose = bb_to_centroid(pcl_msg, *seatable.xywh)
                        return 'got_available_seat'
            
            elif seatable.name in ["sofa", "bench"]:
                
                people_in_sofa = 0
                free_space = seatable.xywh[0], seatable.xywh[0] + seatable.xywh[2]

                for person in people:

                    if self.bb_intersects(seatable.xywh, person.xywh) and self.get_percentage_overlap(seatable.xywh, person.xywh) > 0.7:
                        
                        people_in_sofa+=1

                        if person.xywh[0] > np.mean(free_space):
                            free_space = seatable.xywh[0], np.mean(free_space)
                        else:
                            free_space = np.mean(free_space), seatable.xywh[2]

                if people_in_sofa < PEOPLE_PER_SOFA:
                    userdata.seatable_pose = bb_to_centroid(
                        pcl_msg,
                        free_space[0],
                        seatable.xywh[1],
                        free_space[1] - free_space[0],
                        seatable.xywh[3]
                    )
                    return 'got_available_seat'

        return 'no_available_seat'