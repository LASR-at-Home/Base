#!/usr/bin/env python3
import rospy
import smach

from lasr_vision_msgs.srv import YoloDetection
from common_math import pcl_msg_to_cv2, seg_to_centroid
from cv_bridge3 import CvBridge
from coffee_shop.srv import TfTransform, TfTransformRequest
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point
import numpy as np


class DetectPeople3D(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['pcl_msg'], output_keys=['people_detections_3d'])
        self.yolo = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
        self.tf = rospy.ServiceProxy("/tf_transform", TfTransform)
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
        try:
            cv_im = pcl_msg_to_cv2(userdata.pcl_msg)
            img_msg = self.bridge.cv2_to_imgmsg(cv_im)
            result = self.yolo(img_msg, "yolov8n-seg.pt", 0.5, 0.3)
            result.detected_objects = [det for det in result.detected_objects if det.name == "person"]
            result = [(detection, self.estimate_pose(userdata.pcl_msg, detection)) for detection in result.detected_objects]
            userdata.people_detections_3d = result
            return 'succeeded'
        except rospy.ServiceException as e:
            rospy.logwarn(f"Unable to perform inference. ({str(e)})")
            return 'failed'