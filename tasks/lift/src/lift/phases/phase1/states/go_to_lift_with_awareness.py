#!/usr/bin/env python3
import os

import smach
from pal_startup_msgs.srv import StartupStart, StartupStop
import rosservice
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped, PointCloud2
from std_msgs.msg import String
from tf_module.srv import TfTransform, TfTransformRequest
import numpy as np
import ros_numpy as rnp

class GoToLiftWithAwareness(smach.State):
    def __init__(self, controllers, voice, tf):
        smach.State.__init__(self, outcomes=['success'])

        self.controllers = controllers
        self.voice = voice
        self.tf = tf

        # stop head manager
        if "/pal_startup_control/stop" in rosservice.get_service_list():
            self.stop_head_manager = rospy.ServiceProxy("/pal_startup_control/stop", StartupStop)
            self.start_head_manager = rospy.ServiceProxy("/pal_startup_control/start", StartupStart)

    def estimate_pose(self, pcl_msg, cv_im, detection):
        contours = np.array(detection.xyseg).reshape(-1, 2)
        mask = np.zeros((cv_im.shape[0], cv_im.shape[1]), np.uint8)
        # cv2.fillPoly(mask, pts=[contours], color=(255, 255, 255))
        indices = np.argwhere(mask)
        if indices.shape[0] == 0:
            return np.array([np.inf, np.inf, np.inf])
        pcl_xyz = rnp.point_cloud2.pointcloud2_to_xyz_array(pcl_msg, remove_nans=False)

        xyz_points = []
        for x, y in indices:
            x, y, z = pcl_xyz[x][y]
            xyz_points.append([x, y, z])

        x, y, z = np.nanmean(xyz_points, axis=0)
        centroid = PointStamped()
        centroid.point = Point(x, y, z)
        centroid.header = pcl_msg.header
        tf_req = TfTransformRequest()
        tf_req.target_frame = String("map")
        tf_req.point = centroid
        response = self.tf(tf_req)
        return np.array([response.target_point.point.x, response.target_point.point.y, response.target_point.point.z])

    def execute(self, userdata):
        # result = self.controllers.base_controller.sync_to_pose(get_pose_from_param('/lift/pose'))
        # get the point from zoe

        # stop head manager
        hm = self.stop_head_manager.call("head_manager")


        # face the lift direction

        # search for a person
        pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        cv_im = pcl_msg_to_cv2(pcl_msg)
        img_msg = self.bridge.cv2_to_imgmsg(cv_im)
        detections = self.detect(img_msg, "yolov8n-seg.pt", 0.3, 0.3)
        detections = [(det, self.estimate_pose(pcl_msg, cv_im, det)) for det in detections.detected_objects if
                      det.name == "person"]

        # get the person position
        for i, det in enumerate(detections):
            rospy.set_param("/person/position", det[i][1].tolist())

        # start head manager
        hm = self.start_head_manager.call("head_manager", '')

        # navigate to person
        pose = rospy.get_param("/person/pose")
        self.controllers.base_controller.sync_to_radius(pose[0], pose[1], radius=2.5)

        # face the person
        self.controllers.base_controller.sync_face_to(pose[0], pose[1])


        return 'success'
