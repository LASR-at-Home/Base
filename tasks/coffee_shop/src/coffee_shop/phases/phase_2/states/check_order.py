#!/usr/bin/env python3
import smach
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
from common_math import pcl_msg_to_cv2, seg_to_centroid
from coffee_shop.srv import TfTransform, TfTransformRequest
from play_motion_msgs.msg import PlayMotionGoal
from collections import Counter
from control_msgs.msg import PointHeadGoal
from geometry_msgs.msg import Point


class CheckOrder(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['correct', 'incorrect'])
        self.context = context
        self.n_checks = 0

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

    def execute(self, userdata):

        if self.n_checks == 3:
            self.context.voice_controller.sync_tts("I think I have something in my eyes, I'm struggling to check the order. I trust you that the order is correct!")
            return 'correct'

        self.n_checks += 1

        pm_goal = PlayMotionGoal(motion_name="check_table_low", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)

        order = self.context.tables[self.context.current_table]["order"]
        counter_corners = rospy.get_param(f"/counter/cuboid")
        """
        pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)
        counter_centroid = np.mean(counter_corners, axis=0)
        ph_goal = PointHeadGoal()
        ph_goal.max_velocity = 1.0
        ph_goal.pointing_frame = 'head_2_link'
        ph_goal.pointing_axis = Point(1.0, 0.0, 0.0)
        ph_goal.target.header.frame_id = 'map'
        ph_goal.target.point = Point(*counter_centroid, 0.7)
        self.context.point_head_client.send_goal_and_wait(ph_goal)
        """
        rospy.sleep(rospy.Duration(2.0))
        pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        cv_im = pcl_msg_to_cv2(pcl_msg)
        img_msg = self.context.bridge.cv2_to_imgmsg(cv_im)
        detections = self.context.yolo(img_msg, self.context.YOLO_counter_model, 0.6, 0.3)
        detections = [(det, self.estimate_pose(pcl_msg, det)) for det in detections.detected_objects if det.name in self.context.target_object_remappings.keys()]
        satisfied_points = self.context.shapely.are_points_in_polygon_2d(counter_corners, [[pose[0], pose[1]] for (_, pose) in detections]).inside
        given_order = [detections[i] for i in range(0, len(detections)) if satisfied_points[i]]
        rospy.loginfo(detections)
        for _, pose in detections:
            self.context.publish_object_pose(*pose, "map")

        given_order = [detection[0].name for detection in given_order]
        given_order[:] = [x if x != "biscuits" else "granola" for x in given_order]

        if sorted(order) == sorted(given_order):
            return 'correct'

        missing_items = list((Counter(order) - Counter(given_order)).elements())
        missing_items_string = ', '.join([f"{count} {self.context.target_object_remappings[item] if count == 1 else self.context.target_object_remappings[item]+'s'}" for item, count in Counter(missing_items).items()]).replace(', ', ', and ', len(missing_items) - 2)
        invalid_items = list((Counter(given_order) - Counter(order)).elements())
        invalid_items_string = ', '.join([f"{count} {self.context.target_object_remappings[item] if count == 1 else self.context.target_object_remappings[item]+'s'}" for item, count in Counter(invalid_items).items()]).replace(', ', ', and ', len(invalid_items) - 2)
        rospy.loginfo(f"Order: {order}, Given order: {given_order}")

        if not len(invalid_items):
            self.context.voice_controller.sync_tts(f"You didn't give me {missing_items_string} which I asked for. Please correct the order.")
        elif not len(missing_items):
            self.context.voice_controller.sync_tts(f"You have given me {invalid_items_string} which I didn't ask for. Please correct the order.")
        else:
            self.context.voice_controller.sync_tts(f"You have given me {invalid_items_string} which I didn't ask for, and didn't give me {missing_items_string} which I asked for. Please correct the order.")
        return 'incorrect'
