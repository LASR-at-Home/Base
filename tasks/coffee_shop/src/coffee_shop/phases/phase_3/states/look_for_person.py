#!/usr/bin/env python3
import smach
import rospy
import numpy as np
import ros_numpy as rnp
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
import cv2
from coffee_shop.srv import TfTransform, TfTransformRequest
from common_math import pcl_msg_to_cv2
from play_motion_msgs.msg import PlayMotionGoal

class LookForPerson(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['found', 'not found'])
        self.context = context

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
        tf_req.point = centroid
        response = self.context.tf(tf_req)
        return np.array([response.target_point.point.x, response.target_point.point.y, response.target_point.point.z])

    def execute(self, userdata):
        self.context.stop_head_manager("head_manager")

        pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)

        corners = rospy.get_param("/wait/cuboid")
        pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        cv_im = pcl_msg_to_cv2(pcl_msg)
        img_msg = self.context.bridge.cv2_to_imgmsg(cv_im)
        detections = self.context.yolo(img_msg, self.context.YOLO_person_model, 0.3, 0.3)
        detections = [(det, self.estimate_pose(pcl_msg, cv_im, det)) for det in detections.detected_objects if det.name == "person"]
        satisfied_points = self.context.shapely.are_points_in_polygon_2d(corners, [[pose[0], pose[1]] for (_, pose) in detections]).inside
        if len(detections):
            for i in range(0, len(detections)):
                pose = detections[i][1]
                self.context.publish_person_pose(*pose, "map")
                if satisfied_points[i]:
                    self.context.new_customer_pose = pose.tolist()
                    return 'found'
        rospy.sleep(rospy.Duration(1.0))

        self.context.start_head_manager("head_manager", '')

        return 'not found'