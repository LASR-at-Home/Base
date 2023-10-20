#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import PointCloud2
from play_motion_msgs.msg import PlayMotionGoal
import numpy as np
from common_math import pcl_msg_to_cv2

from std_msgs.msg import String
from play_motion_msgs.msg import PlayMotionGoal
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point
from common_math import pcl_msg_to_cv2, seg_to_centroid
from coffee_shop.srv import TfTransform, TfTransformRequest
import numpy as np

class GuidePerson(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['done'])
        self.context = context

    def estimate_pose(self, pcl_msg, detection):
        centroid_xyz = seg_to_centroid(pcl_msg, np.array(detection.xyseg))
        centroid = PointStamped()
        centroid.point = Point(*centroid_xyz)
        centroid.header = pcl_msg.header
        tf_req = TfTransformRequest()
        tf_req.target_frame = String("map")
        tf_req.point = centroid
        response = self.context.tf(tf_req)
        return np.array([response.target_point.point.x, response.target_point.point.y, response.target_point.point.z])


    def perform_detection(self, pcl_msg, polygon, filter, model):
        cv_im = pcl_msg_to_cv2(pcl_msg)
        img_msg = self.context.bridge.cv2_to_imgmsg(cv_im)
        detections = self.context.yolo(img_msg, model, 0.5, 0.3)
        detections = [(det, self.estimate_pose(pcl_msg, det)) for det in detections.detected_objects if det.name in filter]
        rospy.loginfo(f"All: {[(det.name, pose) for det, pose in detections]}")
        rospy.loginfo(f"Boundary: {polygon}")
        satisfied_points = self.context.shapely.are_points_in_polygon_2d(polygon, [[pose[0], pose[1]] for (_, pose) in detections]).inside
        detections = [detections[i] for i in range(0, len(detections)) if satisfied_points[i]]
        rospy.loginfo(f"Filtered: {[(det.name, pose) for det, pose in detections]}")
        return detections

    def execute(self, userdata):
        self.context.stop_head_manager("head_manager")
        robot_x, robot_y = self.context.base_controller.get_pose()
        utter_clean_phrase = False
        empty_tables = [(label, rospy.get_param(f"/tables/{label}")) for label, table in self.context.tables.items() if table["status"] == "ready"]
        if not empty_tables:
            needs_cleaning_tables = [(label, rospy.get_param(f"/tables/{label}")) for label, table in self.context.tables.items() if table["status"] == "needs cleaning"]
            if needs_cleaning_tables:
                closest_table = min(needs_cleaning_tables, key=lambda table: np.linalg.norm([table[1]["location"]["position"]["x"] - robot_x, table[1]["location"]["position"]["y"] - robot_y]))
                utter_clean_phrase = True
            else:
                self.context.voice_controller.sync_tts("Sorry, we don't have any free tables at the moment. Please come back later.")
                return 'done'
        else:
            closest_table = min(empty_tables, key=lambda table: np.linalg.norm([table[1]["location"]["position"]["x"] - robot_x, table[1]["location"]["position"]["y"] - robot_y]))

        label, table = closest_table
        self.context.current_table = label
        position, orientation = table["location"]["position"], table["location"]["orientation"]
        self.context.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))

        pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)

        if utter_clean_phrase:
            self.context.voice_controller.sync_tts("Please be seated, I will wait for you to sit down! Someone will come to clean this table for you")
        else:
            self.context.voice_controller.sync_tts("Please be seated, I will wait for you to sit down!")
            
        self.person_polygon = rospy.get_param(f"/tables/{self.context.current_table}/persons_cuboid")

        motions = ["look_left", "look_right"]
        customer_seated = False
        for _ in range(5):
            for motion in motions:
                pm_goal = PlayMotionGoal(motion_name=motion, skip_planning=True)
                self.context.play_motion_client.send_goal_and_wait(pm_goal)
                pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
                customer_seated = len(self.perform_detection(pcl_msg, self.person_polygon, ["person"], self.context.YOLO_person_model)) > 0
                if customer_seated:
                    break
            if customer_seated:
                break

        pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)

        self.context.tables[label]["status"] = "needs serving"
        self.context.start_head_manager("head_manager", '')
        return 'done'
