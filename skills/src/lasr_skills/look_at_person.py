#!/usr/bin/env python3

import smach_ros
from geometry_msgs.msg import Point, PointStamped
import smach
from vision import Get3DImage
from lasr_vision_msgs.srv import BodyPixDetection, BodyPixDetectionRequest
from lasr_vision_msgs.msg import BodyPixMaskRequest
from lasr_skills import LookToPoint
import cv2
import rospy
from sensor_msgs.msg import Image
from cv2_img import cv2_img_to_msg, msg_to_cv2_img
from markers import create_and_publish_marker
from visualization_msgs.msg import Marker
from cv2_pcl import pcl_to_img_msg
import ros_numpy as rnp

DEBUG = True

class CheckEyes(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed"], input_keys=["poses", "img_msg", "masks"],
                             output_keys=["pointstampted"])
        self.img_pub = rospy.Publisher("/debug/image", Image, queue_size=1)
        self.marker_pub = rospy.Publisher("eyes", Marker, queue_size=1)

    def execute(self, userdata):
        # the img msg is pcl
        if userdata.poses is None:
            return "failed"

        left_eye, right_eye, eye_point = [], [], []

        for pose in userdata.poses:
            for keypoint in pose.keypoints:
                if keypoint.part == "leftEye":
                    left_eye = keypoint.xy
                if keypoint.part == "rightEye":
                    right_eye = keypoint.xy

        if DEBUG:
            img = msg_to_cv2_img(pcl_to_img_msg(userdata.img_msg))

            cv2.circle(img, (left_eye[0], left_eye[1]), 5, (0, 255, 0), -1)
            cv2.circle(img, (right_eye[0], right_eye[1]), 5, (0, 255, 0), -1)
            img_msg1 = cv2_img_to_msg(img)

            self.img_pub.publish(img_msg1)

        if len(left_eye) == 0 or len(right_eye) == 0:
            return "failed"

        if len(left_eye) == 2 and len(right_eye) == 2:
            eye_point = (left_eye[0] + right_eye[0]) / 2, (left_eye[1] + right_eye[1]) / 2

        if len(left_eye) == 2 and len(right_eye) == 0:
            eye_point = left_eye

        elif len(left_eye) == 0 and len(right_eye) == 2:
            eye_point = right_eye

        pcl_xyz = rnp.point_cloud2.pointcloud2_to_xyz_array(userdata.img_msg, remove_nans=False)
        eye_point_pcl = pcl_xyz[int(eye_point[1]), int(eye_point[0])]

        look_at = PointStamped()
        look_at.header = userdata.img_msg.header
        look_at.point.x = eye_point_pcl[0]
        look_at.point.y = eye_point_pcl[1]
        look_at.point.z = eye_point_pcl[2]

        create_and_publish_marker(self.marker_pub, look_at, r=0, g=1, b=0)

        userdata.pointstampted = look_at

        return "succeeded"


class IncreaseTorsoHeight(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

    def execute(self, userdata):
        return "succeeded"


class LookAtPerson(smach.StateMachine):

    def __init__(self):
        super(LookAtPerson, self).__init__(outcomes=["succeeded", "failed"], input_keys=[],
                                           output_keys=["masks", "poses", "pointstampted"])

        with self:
            smach.StateMachine.add(
                "GET_IMAGE",
                Get3DImage("/xtion/depth_registered/points"),
                transitions={
                    "succeeded": "SEGMENT_PERSON",
                },
                remapping={"img_msg": "img_msg"},
            )

            eyes = BodyPixMaskRequest()
            eyes.parts = ['left_eye', 'right_eye']
            masks = [eyes]

            smach.StateMachine.add(
                "SEGMENT_PERSON",
                smach_ros.ServiceState(
                    "/bodypix/detect",
                    BodyPixDetection,
                    request_cb=lambda ud, _: BodyPixDetectionRequest(pcl_to_img_msg(ud.img_msg), "resnet50", 0.7,
                                                                     masks),
                    response_slots=['masks', 'poses'],
                    input_keys=["img_msg"],
                    output_keys=["masks", "poses"],
                ),
                transitions={
                    "succeeded": "CHECK_EYES",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )
            smach.StateMachine.add(
                "CHECK_EYES",
                CheckEyes(),
                transitions={
                    "succeeded": "LOOK_TO_POINT",
                    "failed": "failed",
                },
                remapping={"img_msg": "img_msg", "poses": "poses", "pointstampted": "pointstampted"}
            )

            smach.StateMachine.add(
                "LOOK_TO_POINT",
                LookToPoint(),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"point": "point"},
            )
