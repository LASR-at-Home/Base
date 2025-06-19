#!/usr/bin/env python3.10

import smach
import rospy
import numpy as np
import cv2
import torch
import os
import actionlib
import tf
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from message_filters import Subscriber, ApproximateTimeSynchronizer

from segment_anything import sam_model_registry, SamPredictor
from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import PointStamped, Point

from lasr_vision_msgs.srv import LangSamRequest
from lasr_vision_msgs.srv import LangSam


class GoToBag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        # --- Model and CV ---
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.depth_info = None
        self.click_pixel = None

        cv2.namedWindow("Click to segment", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Click to segment", self.on_mouse_click)

        # Subscribers
        rgb_sub = Subscriber("/xtion/rgb/image_raw", Image)
        depth_sub = Subscriber("/xtion/depth/image_raw", Image)
        info_sub = Subscriber("/xtion/depth/camera_info", CameraInfo)
        ats = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub], queue_size=5, slop=0.1
        )
        ats.registerCallback(self.synced_callback)

        # move_base client
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base…")
        self.move_base.wait_for_server()
        rospy.loginfo("Connected to move_base")

        # TF
        self.tf_listener = tf.TransformListener()

        # --- AMCL pose ---
        self.latest_amcl_pose = None
        rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback
        )

        self.point_head_client = actionlib.SimpleActionClient(
            "/head_controller/point_head_action", PointHeadAction
        )
        self.point_head_client.wait_for_server()

        rospy.wait_for_service("/lasr_vision/lang_sam")
        self.langsam_srv = rospy.ServiceProxy("/lasr_vision/lang_sam", LangSam)

    def amcl_pose_callback(self, msg):
        self.latest_amcl_pose = msg

    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_pixel = (x, y)
            rospy.loginfo(f"User clicked: {self.click_pixel}")

    def synced_callback(self, rgb_msg, depth_msg, info_msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            self.latest_depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
            self.depth_info = info_msg
        except Exception as e:
            rospy.logerr(f"CV bridge error: {e}")

    def execute(self, userdata):
        # Wait for image & depth as before
        while not rospy.is_shutdown():
            if (
                self.latest_rgb is not None
                and self.latest_depth is not None
                and self.depth_info is not None
            ):
                break
            rospy.sleep(0.1)

        # Prepare the request (assuming self.latest_rgb is a cv2 image)
        rgb_msg = self.bridge.cv2_to_imgmsg(self.latest_rgb, "bgr8")
        langsam_req = LangSamRequest()
        langsam_req.image_raw = rgb_msg
        langsam_req.prompt = "bag"

        try:
            resp = self.langsam_srv(langsam_req)
        except rospy.ServiceException as e:
            rospy.logerr(f"LangSAM service call failed: {e}")
            return "failed"

        if not resp.detections:
            rospy.logwarn("No bags detected.")
            cv2.destroyAllWindows()
            return "failed"

        # Pick detection with highest detection_score
        best_det = max(resp.detections, key=lambda det: det.detection_score)
        mask_flat = np.array(best_det.seg_mask, dtype=np.uint8)
        height, width = self.latest_rgb.shape[:2]
        if mask_flat.size != height * width:
            rospy.logerr(
                f"Mask size {mask_flat.size} does not match image size {height}x{width}"
            )
            cv2.destroyAllWindows()
            return "failed"
        mask = mask_flat.reshape((height, width))

        mu, mv = self.get_mask_median(mask)
        if mu is None or mv is None:
            rospy.logwarn("No valid median pixel in mask.")
            cv2.destroyAllWindows()
            return "failed"

        # Optionally, publish a debug overlay as before
        overlay = self._make_overlay(self.latest_rgb, mask)
        cv2.imshow("Auto-segmented bag", overlay)
        cv2.waitKey(1)

        # Use the rest of your pipeline as before
        p_cam = self.pixel_to_3d(mu, mv)
        if p_cam is None:
            rospy.logwarn("No valid depth at median pixel.")
            cv2.destroyAllWindows()
            return "failed"

        head_point = self.transform_from_camera_to_map(p_cam)
        x_bag, y_bag = self.navigate_to_closest_feasible_point(p_cam)
        if x_bag is not None and y_bag is not None:
            rospy.loginfo(
                "[LangSAM] Navigation succeeded! Adjusting orientation with AMCL pose."
            )
            self.face_point_with_amcl(x_bag, y_bag)
            self.look_at_point(head_point, "map")
            cv2.destroyAllWindows()
            return "succeeded"
        else:
            cv2.destroyAllWindows()
            return "failed"

    # --- all your existing helper methods (unchanged, just copy from your node) ---
    def get_click_mask(self, rgb_bgr, u, v):
        img_rgb = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB)
        self.predictor.set_image(img_rgb)
        point_coords = np.array([[u, v]])
        point_labels = np.array([1])
        masks, _, _ = self.predictor.predict(
            point_coords=point_coords, point_labels=point_labels, multimask_output=True
        )
        H, W = rgb_bgr.shape[:2]
        combined = np.zeros((H, W), dtype=np.uint8)
        for m in masks:
            combined = np.logical_or(combined, m).astype(np.uint8)
        # Largest connected component
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            combined, connectivity=8
        )
        if num_labels <= 1:
            return np.zeros((H, W), dtype=np.uint8)
        areas = stats[1:, cv2.CC_STAT_AREA]
        largest_label = 1 + np.argmax(areas)
        mask = (labels == largest_label).astype(np.uint8)
        return mask

    def get_mask_median(self, mask):
        ys, xs = np.where(mask)
        if len(xs) == 0 or len(ys) == 0:
            return None, None
        u = int(np.median(xs))
        v = int(np.median(ys))
        return u, v

    def pixel_to_3d(self, u, v):
        raw_depth_mm = float(self.latest_depth[v, u])
        if not np.isfinite(raw_depth_mm) or raw_depth_mm <= 0.0:
            return None
        z_cam = raw_depth_mm * 0.001  # meters
        if z_cam > 50.0:
            return None
        fx = self.depth_info.K[0]
        fy = self.depth_info.K[4]
        cx = self.depth_info.K[2]
        cy = self.depth_info.K[5]
        x_cam = (u - cx) * z_cam / fx
        y_cam = (v - cy) * z_cam / fy
        return np.array([x_cam, y_cam, z_cam])

    def transform_from_camera_to_map(self, p_cam):
        cam_frame = "xtion_depth_optical_frame"

        # Create the PointStamped in camera frame
        target_pt = PointStamped()
        target_pt.header.stamp = rospy.Time(0)  # Use latest available
        target_pt.header.frame_id = cam_frame
        target_pt.point.x = float(p_cam[0])
        target_pt.point.y = float(p_cam[1])
        target_pt.point.z = float(p_cam[2])

        # Transform to map frame
        map_frame = "map"
        try:
            self.tf_listener.waitForTransform(
                map_frame, cam_frame, rospy.Time(0), rospy.Duration(1.0)
            )
            target_in_head = self.tf_listener.transformPoint(map_frame, target_pt)
            return [
                target_in_head.point.x,
                target_in_head.point.y,
                target_in_head.point.z,
            ]
        except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
            rospy.logerr("TF transform for look_at_point failed: %s", e)
            return

    def navigate_to_closest_feasible_point(self, p_cam):
        cam_frame = self.depth_info.header.frame_id  # e.g. "xtion_depth_optical_frame"
        try:
            self.tf_listener.waitForTransform(
                "map", cam_frame, rospy.Time(0), rospy.Duration(1.0)
            )
            (trans, rot) = self.tf_listener.lookupTransform(
                "map", cam_frame, rospy.Time(0)
            )
        except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
            rospy.logerr(f"TF lookup failed: {e}")
            return (None, None)

        T = tf.transformations.quaternion_matrix(rot)
        T[0:3, 3] = trans
        p_cam_h = np.array([p_cam[0], p_cam[1], p_cam[2], 1.0])
        p_map_h = T.dot(p_cam_h)
        p_map = p_map_h[:3]
        x_bag, y_bag = p_map[0], p_map[1]

        if np.linalg.norm(p_map[:2]) > 100.0:
            rospy.logwarn(f"Mapped bag center too far: {p_map[:2]}")
            return (None, None)

        # Get robot's current position
        try:
            self.tf_listener.waitForTransform(
                "map", "base_link", rospy.Time(0), rospy.Duration(1.0)
            )
            (robot_trans, _) = self.tf_listener.lookupTransform(
                "map", "base_link", rospy.Time(0)
            )
            x_robot, y_robot = robot_trans[0], robot_trans[1]
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            x_robot, y_robot = x_bag, y_bag  # fallback to bag position

        dx = x_robot - x_bag
        dy = y_robot - y_bag
        dist = np.hypot(dx, dy)
        if dist < 1e-4:
            dx, dy = 1.0, 0.0
            dist = 1.0
        ux = dx / dist
        uy = dy / dist

        # Try moving to increasing distances back from the bag, up to 2m away
        approach_min = 0.2  # minimum approach distance (meters)
        approach_max = min(2.0, dist - 0.05)  # max backoff, never behind robot
        step = 0.05  # meters

        for approach_dist in np.arange(approach_min, approach_max, step):
            goal_x = x_bag + approach_dist * ux
            goal_y = y_bag + approach_dist * uy
            goal_z = 0.0

            # Face the bag
            vx = x_bag - goal_x
            vy = y_bag - goal_y
            yaw = np.arctan2(vy, vx)

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = goal_x
            goal.target_pose.pose.position.y = goal_y
            goal.target_pose.pose.position.z = goal_z

            quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
            goal.target_pose.pose.orientation.x = quat[0]
            goal.target_pose.pose.orientation.y = quat[1]
            goal.target_pose.pose.orientation.z = quat[2]
            goal.target_pose.pose.orientation.w = quat[3]

            rospy.loginfo(
                f"[SAM-CLICK] Trying to navigate to ({goal_x:.2f}, {goal_y:.2f}) dist {approach_dist:.2f} from bag"
            )
            self.move_base.send_goal(goal)
            ok = self.move_base.wait_for_result(rospy.Duration(12.0))
            if ok and self.move_base.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(
                    "[SAM-CLICK] Navigation succeeded at closest feasible point!"
                )
                return (
                    x_bag,
                    y_bag,
                )  # return the true target for use in face_point_with_amcl
            else:
                rospy.logwarn(
                    "[SAM-CLICK] Navigation failed at this distance, trying further away..."
                )

        rospy.logerr("[SAM-CLICK] Could not find any feasible approach point.")
        return (None, None)

    def face_point_with_amcl(self, target_x, target_y):
        rospy.loginfo(
            f"[SAM-CLICK] Request to face ({target_x:.2f}, {target_y:.2f}) using AMCL pose."
        )
        for attempt in range(10):
            if self.latest_amcl_pose is None:
                rospy.logwarn("No AMCL pose available yet, waiting...")
                rospy.sleep(0.2)
                continue

            # Get robot's current pose from AMCL
            pose = self.latest_amcl_pose.pose.pose
            robot_x = pose.position.x
            robot_y = pose.position.y

            # Compute desired yaw to face the target
            dx = target_x - robot_x
            dy = target_y - robot_y
            desired_yaw = np.arctan2(dy, dx)

            # Get current yaw
            quat = [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
            _, _, current_yaw = tf.transformations.euler_from_quaternion(quat)

            # Check if already close enough
            angle_error = np.abs(
                ((desired_yaw - current_yaw + np.pi) % (2 * np.pi)) - np.pi
            )
            if angle_error < 0.05:  # radians (~3 degrees)
                rospy.loginfo("[SAM-CLICK] Already facing target.")
                return True

            # Build a move_base goal to rotate in place
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = robot_x
            goal.target_pose.pose.position.y = robot_y
            goal.target_pose.pose.position.z = 0.0
            q_desired = tf.transformations.quaternion_from_euler(0, 0, desired_yaw)
            goal.target_pose.pose.orientation.x = q_desired[0]
            goal.target_pose.pose.orientation.y = q_desired[1]
            goal.target_pose.pose.orientation.z = q_desired[2]
            goal.target_pose.pose.orientation.w = q_desired[3]

            rospy.loginfo(f"[SAM-CLICK] Rotating in place to yaw {desired_yaw:.2f} rad")
            self.move_base.send_goal(goal)
            ok = self.move_base.wait_for_result(rospy.Duration(6.0))
            if ok and self.move_base.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("[SAM-CLICK] Rotation succeeded!")
                return True
            else:
                rospy.logwarn("[SAM-CLICK] Rotation failed, retrying...")
                rospy.sleep(0.5)
        rospy.logerr("[SAM-CLICK] Failed to adjust orientation after multiple tries.")
        return False

    def _make_overlay(self, rgb_img: np.ndarray, mask: np.ndarray) -> np.ndarray:
        # Overlay mask in red, semi-transparent
        red_mask = np.zeros_like(rgb_img, dtype=np.uint8)
        red_mask[:, :, 2] = (mask * 255).astype(np.uint8)
        alpha = 0.4
        overlay = cv2.addWeighted(rgb_img, 1.0, red_mask, alpha, 0.0)
        return overlay

    def look_at_point(self, target_point, target_frame):
        if not self.point_head_client:
            return

        # If target_point is a numpy array or list, convert to geometry_msgs/Point
        if isinstance(target_point, (list, tuple, np.ndarray)):
            pt = Point()
            pt.x = float(target_point[0])
            pt.y = float(target_point[1])
            pt.z = float(target_point[2])
        else:
            pt = target_point  # already geometry_msgs/Point

        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time(0)
        goal.target.header.frame_id = target_frame
        goal.target.point = pt

        goal.pointing_frame = "xtion_rgb_optical_frame"
        goal.pointing_axis.x = 0.0
        goal.pointing_axis.y = 0.0
        goal.pointing_axis.z = 1.0

        goal.min_duration = rospy.Duration(0.1)
        goal.max_velocity = 1.0

        self.point_head_client.send_goal(goal)
        rospy.loginfo("Looked at point!")


if __name__ == "__main__":
    import smach

    rospy.init_node("go_to_bag_skill_runner")
    sm = smach.StateMachine(outcomes=["succeeded", "failed"])
    with sm:
        smach.StateMachine.add(
            "GO_TO_BAG",
            GoToBag(),
            transitions={"succeeded": "succeeded", "failed": "failed"},
        )
    outcome = sm.execute()
    rospy.loginfo(f"Skill outcome: {outcome}")
