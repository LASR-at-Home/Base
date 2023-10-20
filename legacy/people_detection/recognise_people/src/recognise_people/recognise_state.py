#!/usr/bin/env python3
# coding=utf-8
import os
import shutil
import rospy
from sensor_msgs.msg import Image, PointCloud2
import rospkg
import sys
from face_detection.recognize import recognize
from face_detection.train_model import train
from cv_bridge3 import CvBridge
from cv_bridge3 import cv2
import smach
import itertools

from robocup_receptionist.utils.servers_clients import detect_person
from robocup_receptionist.helpers import get_percentage_overlap, get_corners

from face_detection.srv import FaceDetection, FaceDetectionPCL

'''
   Redetects people in the robot's view using a trained model.
'''
## TODO: refactor

class RedetectState(smach.State):
    def __init__(self, head_controller, torso_controller):
        smach.State.__init__(self, input_keys=["guest_list"], output_keys=['introduction_combinations', "guest_list"],
                             outcomes=['people_recognized', 'no_people_recognized'])
        self.bridge = CvBridge()
        self.head_controller = head_controller
        self.torso_controller = torso_controller

        if self.head_controller:
            self.rotate_head = self.head_controller.sync_reach_to
        else:
            self.rotate_head = lambda *args, **kwargs: None

        # Use PointCloud2
        if rospy.get_published_topics(namespace='/xtion'):
            rospy.wait_for_service("face_detection_pcl")
            self.face_detection = rospy.ServiceProxy("face_detection_pcl", FaceDetectionPCL)
            self.topic = '/xtion/depth_registered/points'
            self.dtype = PointCloud2
        else:
            # Use Image
            rospy.wait_for_service("people_detection")
            self.face_detection = rospy.ServiceProxy("people_detection", FaceDetection)
            self.topic = '/usb_cam/image_raw'
            self.dtype = Image

    def execute(self, userdata):
        print(userdata.guest_list)
        direction = 0.2

        face_detections = []

        yolo_detections = []

        detection_map = {}

        self.torso_controller.sync_raise(0.4)

        for _ in range(4):
            direction *= -1
            self.rotate_head(direction, -0.2, velocities=[0.1, 0.0])

            pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
            face_detection_resp = self.face_detection(pcl_msg)
            yolo_detections.extend(detect_person())

            for detection in face_detection_resp.detections:
                try:
                    n = [d.name for d in face_detections].index(detection.name)
                    if detection.confidence > face_detections[n].confidence:
                        face_detections[n] = detection
                except ValueError:
                    face_detections.append(detection)

        # get bb of person using face bb and body bb
        for face in face_detections:
            for detection in yolo_detections:
                x1, y1, x2, y2 = face.bb

                head_x = (x1 + x2) / 2  # center of head in x-axis
                body_x = (detection.xywh[0] + detection.xywh[2]) / 2  # center of body in x-axis

                if abs(head_x - body_x) < 20:
                    if not face.name in detection_map.keys():
                        detection_map[face.name] = (face, detection)
                        print("yolo detection here, bb overlapped.")
                        break
                    if detection.confidence > detection_map[face.name][1].confidence:
                        detection_map[face.name] = (face, detection)
                else:
                    print(face.name, face.bb, detection.xywh)

        intersect = []
        if len(detection_map.keys()):
            for face_detection, yolo_detection in detection_map.values():
                print(f"Checking {face_detection.name}")
                try:
                    n = [g.name for g in userdata.guest_list].index(face_detection.name)
                    userdata.guest_list[n].last_known_pose = yolo_detection.centroid
                    intersect.append(userdata.guest_list[n])
                except ValueError:
                    continue
                print(intersect)

                # Create list of combinations of pairs of guests.
                comb = list(itertools.combinations([g for g in intersect], 2))

                # Sort the combinations.
                comb = sorted(comb, key=lambda element: (element[0].name, element[1].name))

                # Insert the reverse of each pair, ensuring the sorting critera is maintained.
                comb = [a for pair in [(c, c[::-1]) for c in comb] for a in pair]

                print(comb)
                userdata.introduction_combinations = comb
                return 'people_recognized'
        else:
            return 'no_people_recognized'


if __name__ == '__main__':
    rospy.init_node('smach_example_state_machine')
    sm = smach.StateMachine(outcomes=['success'])
    with sm:
        smach.StateMachine.add('REDETECT', RedetectState(),
                               transitions={'finished_redetection': 'success'},
                               remapping={'out_redetected_people': 'redetected_people'})
    outcome = sm.execute()
    rospy.loginfo('I have completed execution with outcome: ')
    rospy.loginfo(outcome)