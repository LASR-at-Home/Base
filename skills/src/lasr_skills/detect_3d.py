#!/usr/bin/env python3
import rospy
import smach

from common_math import pcl_msg_to_cv2, seg_to_centroid
from cv_bridge3 import CvBridge
from tf_module.srv import TfTransform, TfTransformRequest
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
import numpy as np

from lasr_vision_msgs.srv import YoloDetection


class Detect3D(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["depth_topic", "filter"],
            output_keys=["detections_3d"],
        )
        self.yolo = rospy.ServiceProxy("/yolov8/detect", YoloDetection)
        self.tf = rospy.ServiceProxy("/tf_transform", TfTransform)
        self.bridge = CvBridge()
        self.n_published_markers = 0
        self.marker_point_pub = rospy.Publisher("/detect_objects_3d/points", Marker)
        self.marker_text_pub = rospy.Publisher("/detect_objects_3d/labels", Marker)

    def estimate_pose(self, pcl_msg, detection):
        centroid_xyz = seg_to_centroid(pcl_msg, np.array(detection.xyseg))
        centroid = PointStamped()
        centroid.point = Point(*centroid_xyz)
        centroid.header = pcl_msg.header
        tf_req = TfTransformRequest()
        tf_req.target_frame = String("map")
        tf_req.point = centroid
        response = self.tf(tf_req)
        return np.array(
            [
                response.target_point.point.x,
                response.target_point.point.y,
                response.target_point.point.z,
            ]
        )

    def execute(self, userdata):
        pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        try:
            cv_im = pcl_msg_to_cv2(pcl_msg)
            img_msg = self.bridge.cv2_to_imgmsg(cv_im)
            result = self.yolo(img_msg, "yolov8n-seg.pt", 0.5, 0.3)
            result.detected_objects = [
                det for det in result.detected_objects if det.name in userdata.filter
            ]
            result = [
                (detection, self.estimate_pose(pcl_msg, detection))
                for detection in result.detected_objects
            ]
            for detection, point in result:
                marker_point_msg = Marker()
                marker_point_msg.header.frame_id = "map"
                marker_point_msg.header.stamp = rospy.Time.now()
                marker_point_msg.id = self.n_published_markers
                marker_point_msg.type = Marker.SPHERE
                marker_point_msg.action = Marker.ADD
                marker_point_msg.pose.position.x = point[0]
                marker_point_msg.pose.position.y = point[1]
                marker_point_msg.pose.position.z = point[2]
                marker_point_msg.color.r = 1.0
                marker_point_msg.color.g = 0.0
                marker_point_msg.color.b = 0.0
                marker_point_msg.scale.x = 0.1
                marker_point_msg.scale.y = 0.1
                marker_point_msg.scale.z = 0.1
                marker_point_msg.color.a = 1.0
                self.marker_point_pub.publish(marker_point_msg)

                marker_text_msg = Marker()
                marker_text_msg.header.frame_id = "map"
                marker_text_msg.header.stamp = rospy.Time.now()
                marker_text_msg.id = self.n_published_markers
                marker_text_msg.type = Marker.TEXT_VIEW_FACING
                marker_text_msg.action = Marker.ADD
                marker_text_msg.pose.position.x = point[0]
                marker_text_msg.pose.position.y = point[1]
                marker_text_msg.pose.position.z = point[2] + 0.15
                marker_text_msg.text = detection.name
                marker_text_msg.color.r = 1.0
                marker_text_msg.color.g = 0.0
                marker_text_msg.color.b = 0.0
                marker_text_msg.scale.x = 0.1
                marker_text_msg.scale.y = 0.1
                marker_text_msg.scale.z = 0.1
                marker_text_msg.color.a = 1.0
                self.marker_text_pub.publish(marker_text_msg)

                self.n_published_markers += 1

            userdata.detections_3d = result
            return "succeeded"
        except rospy.ServiceException as e:
            rospy.logwarn(f"Unable to perform inference. ({str(e)})")
            return "failed"
