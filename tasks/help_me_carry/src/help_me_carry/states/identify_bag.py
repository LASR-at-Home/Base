#!/usr/bin/env python3.10

import rospy
import math
import actionlib
import numpy as np
import tf
import smach

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from message_filters import Subscriber, ApproximateTimeSynchronizer

from lasr_vision_msgs.srv import (
    YoloPoseDetection3D,
    YoloPoseDetection3DRequest,
)

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class IdentifyBag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.depth_info = None

        # Subscribers: RGB, depth, camera info
        rgb_sub = Subscriber("/xtion/rgb/image_raw", Image)
        depth_sub = Subscriber("/xtion/depth/image_raw", Image)
        info_sub = Subscriber("/xtion/depth/camera_info", CameraInfo)
        ats = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub], queue_size=5, slop=0.1
        )
        ats.registerCallback(self.synced_callback)

        # Service client: YOLOPoseDetection3D
        rospy.wait_for_service("/yolo/detect3d_pose")
        self.detect3d = rospy.ServiceProxy("/yolo/detect3d_pose", YoloPoseDetection3D)

        # Navigation: move_base client
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base…")
        self.move_base.wait_for_server()
        rospy.loginfo("Connected to move_base")

        # TF listener
        self.tf_listener = tf.TransformListener()

    def synced_callback(self, rgb_msg, depth_msg, info_msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            self.latest_depth = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
            self.depth_info = info_msg
        except Exception as e:
            rospy.logerr(f"CV bridge error: {e}")

    def get_human_keypoints_3d(self):
        if (
            self.latest_rgb is None
            or self.latest_depth is None
            or self.depth_info is None
        ):
            return None

        req = YoloPoseDetection3DRequest(
            image_raw=self.bridge.cv2_to_imgmsg(self.latest_rgb, "bgr8"),
            depth_image=self.bridge.cv2_to_imgmsg(self.latest_depth, "32FC1"),
            depth_camera_info=self.depth_info,
            model=rospy.get_param("~model", "yolo11n-pose.pt"),
            confidence=0.5,
        )

        try:
            res = self.detect3d(req)
        except rospy.ServiceException as e:
            rospy.logerr(f"YOLOPoseDetection3D call failed: {e}")
            return None

        if not res.detections:
            return None

        # take first detection
        kp_list = res.detections[0].keypoints
        return {
            kp.keypoint_name: np.array(
                [kp.point.x / 1000, kp.point.y / 1000, kp.point.z / 1000]
            )
            for kp in kp_list
        }

    def find_pointed_pixel(self, origin, direction, max_dist=2.0, step=0.02):
        fx = self.depth_info.K[0]
        fy = self.depth_info.K[4]
        cx = self.depth_info.K[2]
        cy = self.depth_info.K[5]

        h, w = self.latest_depth.shape[:2]

        for d in np.arange(0, max_dist, step):
            pt = origin + direction * d

            z = pt[2]
            if not np.isfinite(z) or z <= 0.0:
                continue

            u_f = (pt[0] * fx) / z + cx
            v_f = (pt[1] * fy) / z + cy

            u = int(np.floor(u_f))
            v = int(np.floor(v_f))
            if u < 0 or u >= w or v < 0 or v >= h:
                continue

            depth = self.latest_depth[v, u]
            if np.isfinite(depth) and (depth + 0.02) < z:
                return (u, v, pt)

        return None

    def navigate_to(self, cam_point, approach_dist=0.5):
        try:
            self.tf_listener.waitForTransform(
                "map",
                self.depth_info.header.frame_id,
                rospy.Time(0),
                rospy.Duration(1.0),
            )
            (trans, rot) = self.tf_listener.lookupTransform(
                "map", self.depth_info.header.frame_id, rospy.Time(0)
            )
        except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
            rospy.logerr(f"TF lookup failed: {e}")
            return False
        T = tf.transformations.quaternion_matrix(rot)
        T[0:3, 3] = trans
        p_cam = np.array([cam_point[0], cam_point[1], cam_point[2], 1.0])
        p_map = T.dot(p_cam)[:3]
        forward_cam = np.array([0, 0, 1, 0])
        forward_map = T.dot(forward_cam)[:3]
        forward_map /= np.linalg.norm(forward_map)
        approach_pt = p_map - forward_map * approach_dist
        dx, dy = p_map[0] - approach_pt[0], p_map[1] - approach_pt[1]
        yaw = math.atan2(dy, dx)
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = approach_pt[0]
        goal.target_pose.pose.position.y = approach_pt[1]
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]
        self.move_base.send_goal(goal)
        ok = self.move_base.wait_for_result(rospy.Duration(30.0))
        if not ok or self.move_base.get_state() != actionlib.GoalStatus.SUCCEEDED:
            rospy.logerr("Navigation failed")
            return False
        return True

    def execute(self, userdata):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.latest_rgb is None or self.depth_info is None:
                rate.sleep()
                continue
            kps = self.get_human_keypoints_3d()
            rospy.loginfo(f"Keypoints: {kps}")

            best_hit = None
            camera_z_axis = np.array([0.0, 0.0, 1.0])

            for side in ["right", "left"]:
                elbow_key = f"{side}_elbow"
                wrist_key = f"{side}_wrist"
                if kps and elbow_key in kps and wrist_key in kps:
                    o = kps[elbow_key]
                    w = kps[wrist_key]
                    vec = w - o
                    norm = np.linalg.norm(vec)
                    if norm < 1e-6:
                        continue
                    dirv = vec / norm
                    hit = self.find_pointed_pixel(o, dirv)
                    if hit:
                        best_hit = (side, o, dirv, hit)
                        break

            if best_hit is None:
                best_score = -1.0
                best_data = None
                for side in ["right", "left"]:
                    elbow_key = f"{side}_elbow"
                    wrist_key = f"{side}_wrist"
                    if kps and elbow_key in kps and wrist_key in kps:
                        o = kps[elbow_key]
                        w = kps[wrist_key]
                        vec = w - o
                        norm = np.linalg.norm(vec)
                        if norm < 1e-6:
                            continue
                        dirv = vec / norm
                        score = float(np.dot(dirv, camera_z_axis))
                        if score > best_score and score > 0.5:
                            best_score = score
                            best_data = (side, o, dirv)
                if best_data:
                    side, o, dirv = best_data
                    hit = self.find_pointed_pixel(o, dirv)
                    if hit:
                        best_hit = (side, o, dirv, hit)

            if best_hit:
                side, origin, direction, (u, v, pt_cam) = best_hit
                rospy.loginfo(f"Using {side} arm.  Hit at pixel {(u,v)} → {pt_cam}")
                if self.navigate_to(pt_cam):
                    rospy.loginfo("Navigation complete")
                    return "succeeded"
                else:
                    rospy.logerr("Navigation failed after hit")
                    return "failed"

            rate.sleep()

        return "failed"


if __name__ == "__main__":

    rospy.init_node("identify_bag")
    sm = smach.StateMachine(outcomes=["succeeded", "failed"])
    with sm:
        smach.StateMachine.add(
            "IDENTIFY_BAG",
            IdentifyBag(),
            transitions={"succeeded": "succeeded", "failed": "failed"},
        )
    outcome = sm.execute()
    rospy.loginfo(f"Skill outcome: {outcome}")
