#!/usr/bin/env python3
import smach
import rospy
import rospkg
import os
import shutil
import actionlib
from std_msgs.msg import String
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
from cv_bridge3 import CvBridge, cv2
from lasr_object_detection_yolo.srv import YoloDetection
from lasr_voice.voice import Voice
from pcl_segmentation.srv import SegmentCuboid, Centroid, MaskFromCuboid, SegmentBB
from common_math import pcl_msg_to_cv2, seg_to_centroid
from coffee_shop.srv import TfTransform, TfTransformRequest
import numpy as np
from actionlib_msgs.msg import GoalStatus
import ros_numpy as rnp

from lasr_shapely import LasrShapely
shapely = LasrShapely()

OBJECTS = ["cup", "mug"]


def create_point_marker(x, y, z, idx, frame, text):
    marker_msg = Marker()
    marker_msg.header.frame_id = frame
    marker_msg.header.stamp = rospy.Time.now()
    marker_msg.id = idx
    marker_msg.type = Marker.TEXT_VIEW_FACING
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
    marker_msg.text = text
    return marker_msg

class CheckTable(smach.State):
    def __init__(self, head_controller, voice_controller, debug = False):
        smach.State.__init__(self, outcomes=['not_finished', 'finished'])
        self.head_controller = head_controller
        self.voice_controller = voice_controller
        self.debug = debug
        self.play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
        self.play_motion_client.wait_for_server(rospy.Duration(15.0))
        rospy.wait_for_service("/yolov8/detect", rospy.Duration(15.0))
        self.detect = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
        self.tf = rospy.ServiceProxy("/tf_transform", TfTransform)
        self.bridge = CvBridge()
        self.detections_objects = []
        self.detections_people = []
        self.object_pose_pub = rospy.Publisher("/object_poses", Marker, queue_size=100)
        self.people_pose_pub = rospy.Publisher("/people_poses", Marker, queue_size=100)

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

    def publish_object_points(self):
        for i, (det, point) in enumerate(self.detections_objects):
            marker = create_point_marker(*point, i, "map", f"{det.name}, {det.confidence}")
            self.object_pose_pub.publish(marker)

    def publish_people_points(self):
        for i, (det, point) in enumerate(self.detections_people):
            marker = create_point_marker(*point, i, "map", f"{det.name}, {det.confidence}")
            self.people_pose_pub.publish(marker)

    def filter_detections_by_pose(self, detections, threshold=0.2):
        filtered = []

        for i, (detection, point) in enumerate(detections):
            distances = np.array([np.sqrt(np.sum((point - ref_point) ** 2)) for _, ref_point in filtered])
            if not np.any(distances < threshold):
                filtered.append((detection, point))

        return filtered

    def perform_detection(self, pcl_msg, polygon, filter):
        cv_im = pcl_msg_to_cv2(pcl_msg)
        img_msg = self.bridge.cv2_to_imgmsg(cv_im)
        detections = self.detect(img_msg, "yolov8n-seg.pt", 0.6, 0.3)
        detections = [(det, self.estimate_pose(pcl_msg, det)) for det in detections.detected_objects if det.name in filter]
        rospy.loginfo(f"All: {[(det.name, pose) for det, pose in detections]}")
        rospy.loginfo(f"Boundary: {polygon}")
        satisfied_points = shapely.are_points_in_polygon_2d(polygon, [[pose[0], pose[1]] for (_, pose) in detections])
        detections = [detections[i] for i in range(0, len(detections)) if satisfied_points[i]]
        rospy.loginfo(f"Filtered: {[(det.name, pose) for det, pose in detections]}")
        return detections

    def check(self, pcl_msg):
        self.check_table(pcl_msg)
        self.check_people(pcl_msg)

    def check_table(self, pcl_msg):
        detections_objects_ = self.perform_detection(pcl_msg, self.object_polygon, OBJECTS)
        self.detections_objects.extend(detections_objects_)

    def check_people(self, pcl_msg):
        detections_people_ = self.perform_detection(pcl_msg, self.person_polygon, ["person"])
        self.detections_people.extend(detections_people_)

    def execute(self, userdata):
        self.current_table = rospy.get_param("current_table")
        self.object_debug_images = []
        self.people_debug_images = []

        rospy.loginfo(self.current_table)
        self.object_polygon = rospy.get_param(f"/tables/{self.current_table}/objects_cuboid")
        self.person_polygon = rospy.get_param(f"/tables/{self.current_table}/persons_cuboid")
        self.detections_objects = []
        self.detections_people = []

        motions = ["back_to_default", "check_table", "check_table_low", "look_left", "look_right", "back_to_default"]
        #self.detection_sub = rospy.Subscriber("/xtion/depth_registered/points", PointCloud2, self.check)
        for motion in motions:
            pm_goal = PlayMotionGoal(motion_name=motion, skip_planning=True)
            self.play_motion_client.send_goal_and_wait(pm_goal)
            rospy.sleep(2.0)
            pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
            self.check(pcl_msg)

        status = "unknown"
        if len(self.detections_objects) > 0 and len(self.detections_people) == 0:
            status = "needs cleaning"
        elif len(self.detections_objects) > 0 and len(self.detections_people) > 0:
            status = "served"
        elif len(self.detections_objects) == 0 and len(self.detections_people) > 0:
            status = "needs serving"
        elif len(self.detections_objects) == 0 and len(self.detections_people) == 0:
            status = "ready"

        #self.detection_sub.unregister()
 
        self.detections_objects = self.filter_detections_by_pose(self.detections_objects, threshold=0.1)
        self.detections_people = self.filter_detections_by_pose(self.detections_people, threshold=0.50)

        if self.debug:
            self.publish_object_points()
            self.publish_people_points()

        rospy.set_param(f"/tables/{self.current_table}/status/", status)
        self.voice_controller.sync_tts(f"The status of this table is {status}. There were {len(self.detections_objects)} objects and {len(self.detections_people)} people")
        return 'finished' if len([(label, table) for label, table in rospy.get_param("/tables").items() if table["status"] == "unvisited"]) == 0 else 'not_finished'
