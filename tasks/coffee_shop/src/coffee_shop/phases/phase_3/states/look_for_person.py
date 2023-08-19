#!/usr/bin/env python3
import smach
import rospy
import numpy as np
import ros_numpy as rnp
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
from cv_bridge3 import CvBridge, cv2
from lasr_object_detection_yolo.srv import YoloDetection
from coffee_shop.srv import TfTransform, TfTransformRequest
from visualization_msgs.msg import Marker
from common_math import pcl_msg_to_cv2

from lasr_shapely import LasrShapely
shapely = LasrShapely()

class LookForPerson(smach.State):
    def __init__(self, yolo, tf):
        smach.State.__init__(self, outcomes=['found', 'not found'])
        self.detect = yolo
        self.tf = tf
        self.people_pose_pub = rospy.Publisher("/people_poses", Marker, queue_size=100)
        self.bridge = CvBridge()

    def estimate_pose(self, pcl_msg, cv_im, detection):
        contours = np.array(detection.xyseg).reshape(-1, 2)
        mask = np.zeros((cv_im.shape[0], cv_im.shape[1]), np.uint8)
        cv2.fillPoly(mask, pts=[contours], color=(255, 255, 255))
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
        centroid.point = Point(x,y,z)
        centroid.header = pcl_msg.header
        tf_req = TfTransformRequest()
        tf_req.target_frame = String("map")
        tf_req.point  = centroid
        response = self.tf(tf_req)
        return np.array([response.target_point.point.x, response.target_point.point.y, response.target_point.point.z])

    def execute(self, userdata):
        corners = rospy.get_param("/wait/cuboid")
        pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        cv_im = pcl_msg_to_cv2(pcl_msg)
        img_msg = self.bridge.cv2_to_imgmsg(cv_im)
        detections = self.detect(img_msg, "yolov8n-seg.pt", 0.3, 0.3)
        detections = [(det, self.estimate_pose(pcl_msg, cv_im, det)) for det in detections.detected_objects if det.name == "person"]
        satisfied_points = shapely.are_points_in_polygon_2d(corners, [[pose[0], pose[1]] for (_, pose) in detections]).inside
        if len(detections):
            for i in range(0, len(detections)):
                pose = detections[i][1]
                marker_msg = Marker()
                marker_msg.header.frame_id = "map"
                marker_msg.header.stamp = rospy.Time.now()
                marker_msg.id = 0
                marker_msg.type = Marker.SPHERE
                marker_msg.action = Marker.ADD
                marker_msg.pose.position.x = pose[0]
                marker_msg.pose.position.y = pose[1]
                marker_msg.pose.position.z = pose[2]
                marker_msg.pose.orientation.w = 1.0
                marker_msg.scale.x = 0.1
                marker_msg.scale.y = 0.1
                marker_msg.scale.z = 0.1
                marker_msg.color.a = 1.0
                marker_msg.color.r = 1.0
                marker_msg.color.g = 0.0
                marker_msg.color.b = 0.0
                self.people_pose_pub.publish(marker_msg)
                if satisfied_points[i]:
                    rospy.set_param("/person/position", pose.tolist())
                    return 'found'
        rospy.sleep(rospy.Duration(1.0))
        return 'not found'
