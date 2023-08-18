#!/usr/bin/env python3
import smach
import rospy
import numpy as np
import ros_numpy as rnp
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
from common_math import pcl_msg_to_cv2, seg_to_centroid
from lasr_object_detection_yolo.srv import YoloDetection
from coffee_shop.srv import TfTransform, TfTransformRequest
from cv_bridge3 import CvBridge, cv2

from lasr_shapely import LasrShapely
shapely = LasrShapely()

OBJECTS = ["cup", "mug", "bowl"]

class CheckOrder(smach.State):
    def __init__(self, voice_controller, yolo, tf, pm):
        smach.State.__init__(self, outcomes=['correct', 'incorrect'])
        self.voice_controller = voice_controller
        self.detect = yolo
        self.tf = tf
        self.play_motion_client = pm
        self.bridge = CvBridge()

    def estimate_pose(self, pcl_msg, detection):
        centroid_xyz = seg_to_centroid(pcl_msg, np.array(detection.xyseg))
        centroid = PointStamped()
        centroid.point = Point(*centroid_xyz)
        centroid.header = pcl_msg.header
        tf_req = TfTransformRequest()
        tf_req.target_frame = String("map")
        tf_req.point = centroid
        response = self.tf(tf_req)
        return np.array([response.target_point.point.x, response.target_point.point.y, response.target_point.point.z])

    def execute(self, userdata):
        order = rospy.get_param(f"/tables/{rospy.get_param('current_table')}/order")

        counter_corners = rospy.get_param(f"/counter/cuboid")

        pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        cv_im = pcl_msg_to_cv2(pcl_msg)
        img_msg = self.bridge.cv2_to_imgmsg(cv_im)
        detections = self.detect(img_msg, "yolov8n-seg.pt", 0.5, 0.3)
        detections = [(det, self.estimate_pose(pcl_msg, det)) for det in detections.detected_objects if det.name in OBJECTS]
        satisfied_points = shapely.are_points_in_polygon_2d(counter_corners, [[pose[0], pose[1]] for (_, pose) in detections]).inside
        given_order = [detections[i][0].name for i in range(0, len(detections)) if satisfied_points[i]]
        rospy.set_param(f"/tables/{rospy.get_param('current_table')}/given_order", given_order)
        return 'correct' if sorted(order) == sorted(given_order) else 'incorrect'