#!/usr/bin/env python3
import rospy
from common_math import pcl_msg_to_cv2, seg_to_centroid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point
from lasr_object_detection_yolo.srv import YoloDetection
from coffee_shop.srv import TfTransform, TfTransformRequest
from visualization_msgs.msg import Marker
from cv_bridge3 import CvBridge
import numpy as np

def create_point_marker(x, y, z, idx):
    marker_msg = Marker()
    marker_msg.header.frame_id = "map"
    marker_msg.header.stamp = rospy.Time.now()
    marker_msg.id = idx
    marker_msg.type = Marker.SPHERE
    marker_msg.action = Marker.ADD
    marker_msg.pose.position.x = x
    marker_msg.pose.position.y = y
    marker_msg.pose.position.z = z
    marker_msg.pose.orientation.w = 1.0
    marker_msg.scale.x = 0.1
    marker_msg.scale.y = 0.1
    marker_msg.scale.z = 0.1
    marker_msg.color.a = 1.0
    marker_msg.color.r = 1.0
    marker_msg.color.g = 0.0
    marker_msg.color.b = 0.0
    return marker_msg

rospy.init_node("test_people_pose")

people_pose_pub = rospy.Publisher("/people_poses", Marker, queue_size=100)

rospy.wait_for_service("/yolov8/detect", rospy.Duration(15.0))
yolo = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
tf = rospy.ServiceProxy("/tf_transform", TfTransform)
bridge = CvBridge()

idx = 0

while not rospy.is_shutdown():
    pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
    cv_im = pcl_msg_to_cv2(pcl_msg)
    img_msg = bridge.cv2_to_imgmsg(cv_im)
    detections = yolo(img_msg, "yolov8n-seg.pt", 0.3, 0.3)
    for detection in detections.detected_objects:
        if detection.name == "person":
            centroid_xyz = seg_to_centroid(pcl_msg, np.array(detection.xyseg))
            centroid = PointStamped()
            centroid.point = Point(*centroid_xyz)
            centroid.header = pcl_msg.header
            tf_req = TfTransformRequest()
            tf_req.target_frame = String("map")
            tf_req.point = centroid
            centroid_tf = tf(tf_req).target_point.point
            people_pose_pub.publish(create_point_marker(centroid_tf.x, centroid_tf.y, centroid_tf.z, idx))
            idx += 1
            prev_xyz = centroid_xyz

    rospy.sleep(rospy.Duration(1.0))