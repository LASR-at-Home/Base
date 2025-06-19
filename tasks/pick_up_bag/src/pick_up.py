#!/usr/bin/env python3.10
import rospy
import math
import actionlib
import numpy as np
import cv2
import tf
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from message_filters import Subscriber, ApproximateTimeSynchronizer
from lasr_vision_msgs.srv import YoloPoseDetection3D, YoloPoseDetection3DRequest

from segment_anything import sam_model_registry, SamPredictor
import torch
import os


class PointAndNavigateNode:
    def __init__(self):
        rospy.init_node("point_and_navigate_sam")

        # --- (A) CV Bridge + placeholders ---
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None  # store raw depth (uint16, in mm)
        self.depth_info = None

        # Create a named window for visualization
        cv2.namedWindow("SAM Segmentation", cv2.WINDOW_NORMAL)

        # --- (B) Subscribers: RGB, depth, camera_info ---
        rgb_sub = Subscriber("/xtion/rgb/image_raw", Image)
        depth_sub = Subscriber("/xtion/depth/image_raw", Image)
        info_sub = Subscriber("/xtion/depth/camera_info", CameraInfo)
        ats = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub], queue_size=5, slop=0.1
        )
        ats.registerCallback(self.synced_callback)

        # --- (C) YOLOPoseDetection3D client ---
        rospy.wait_for_service("/yolo/detect3d_pose")
        self.detect3d = rospy.ServiceProxy("/yolo/detect3d_pose", YoloPoseDetection3D)

        # --- (D) move_base client ---
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base…")
        self.move_base.wait_for_server()
        rospy.loginfo("Connected to move_base")

        # --- (E) TF listener ---
        self.tf_listener = tf.TransformListener()

        # --- (F) Load SAM model once ---
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        root = os.path.dirname(__file__)
        pkg = os.path.dirname(root)
        ckpt = os.path.join(pkg, "models", "sam_vit_b_01ec64.pth")
        sam = sam_model_registry["vit_b"](checkpoint=ckpt)
        sam.to(self.device)
        self.predictor = SamPredictor(sam)

        rospy.loginfo("PointAndNavigateNode (SAM‐only) ready.")

        rospy.sleep(3)
        self.loop()

    def synced_callback(self, rgb_msg, depth_msg, info_msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            # Read depth as raw uint16 (millimeters)
            self.latest_depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
            self.depth_info = info_msg
        except Exception as e:
            rospy.logerr(f"CV bridge error: {e}")

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if (
                self.latest_rgb is None
                or self.depth_info is None
                or self.latest_depth is None
            ):
                rate.sleep()
                continue

            # 1. Detect human keypoints first
            kps_3d = self.get_human_keypoints_3d()
            if kps_3d is None:
                # No person detected, skip segmentation and navigation
                cv2.imshow("SAM Segmentation", self.latest_rgb)
                cv2.waitKey(1)
                rate.sleep()
                continue

            # 2. Project 3D keypoints to 2D pixels
            wrist_3d = kps_3d.get("right_wrist")
            elbow_3d = kps_3d.get("right_elbow")
            if wrist_3d is None or elbow_3d is None:
                rospy.logwarn("Wrist or elbow not detected in keypoints.")
                rate.sleep()
                continue

            wrist_2d = self.project_3d_to_pixel(wrist_3d)
            elbow_2d = self.project_3d_to_pixel(elbow_3d)
            if wrist_2d is None or elbow_2d is None:
                rospy.logwarn("Failed to project wrist/elbow to 2D pixel.")
                rate.sleep()
                continue

            # 3. Probe along the ray, prompting SAM at each pixel
            pointed_pixel = self.find_last_bag_pixel_along_ray(
                wrist_2d, elbow_2d, self.latest_rgb
            )
            if pointed_pixel is None:
                rospy.logwarn("Could not find a bag pixel along the ray.")
                rate.sleep()
                continue

            u, v = pointed_pixel

            # 4. For visualization: prompt SAM at that pixel and overlay
            img_rgb = cv2.cvtColor(self.latest_rgb, cv2.COLOR_BGR2RGB)
            self.predictor.set_image(img_rgb)
            masks, _, _ = self.predictor.predict(
                point_coords=np.array([[u, v]]),
                point_labels=np.array([1]),
                multimask_output=True,
            )
            H, W = self.latest_rgb.shape[:2]
            combined_mask = np.zeros((H, W), dtype=np.uint8)
            for m in masks:
                combined_mask = np.logical_or(combined_mask, m).astype(np.uint8)
            overlay = self._make_overlay(self.latest_rgb, combined_mask)
            # Optionally, draw a circle at (u, v) to show the "prompted" pixel
            cv2.circle(overlay, (u, v), 5, (0, 255, 0), -1)
            cv2.imshow("SAM Segmentation", overlay)
            cv2.waitKey(1)

            # 5. Back-project intersection pixel to 3D in camera frame
            p_cam_centroid = self.pixel_to_3d(u, v)
            if p_cam_centroid is None:
                rospy.logwarn("No valid depth at pointed pixel.")
                rate.sleep()
                continue

            # 6. Transform to map frame and send navigation goal
            success = self.navigate_to_point_in_map(p_cam_centroid)
            if success:
                rospy.loginfo("[loop] Bag navigation succeeded by pointing gesture.")
                return  # exit after one successful navigation

            rate.sleep()

        cv2.destroyWindow("SAM Segmentation")

    def find_last_bag_pixel_along_ray(self, wrist, elbow, rgb_img):
        """
        Walks along the wrist->elbow ray, prompts SAM at each step,
        and returns the last pixel where a 'bag' is found before hitting ground.
        """
        u0, v0 = wrist
        u1, v1 = elbow
        du, dv = u0 - u1, v0 - v1
        norm = np.hypot(du, dv)
        if norm < 1e-4:
            return None
        du, dv = du / norm, dv / norm

        H, W = rgb_img.shape[:2]
        steps = int(2 * max(H, W))

        img_rgb = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2RGB)
        self.predictor.set_image(img_rgb)

        last_bag_pixel = None
        for i in range(steps):
            u = int(round(u0 + du * i))
            v = int(round(v0 + dv * i))
            if not (0 <= u < W and 0 <= v < H):
                break

            # Prompt SAM with this pixel
            masks, _, _ = self.predictor.predict(
                point_coords=np.array([[u, v]]),
                point_labels=np.array([1]),
                multimask_output=True,
            )

            # Combine all returned masks (logical OR)
            combined_mask = np.zeros((H, W), dtype=np.uint8)
            for m in masks:
                combined_mask = np.logical_or(combined_mask, m).astype(np.uint8)

            # If mask is empty at this pixel, we've hit ground/background
            if combined_mask[v, u] == 0:
                break

            # Otherwise, remember this pixel as being on the bag
            last_bag_pixel = (u, v)

        return last_bag_pixel

    def project_3d_to_pixel(self, point_3d):
        """
        point_3d: np.array([x, y, z]) in camera frame (meters)
        Returns: (u, v) pixel in image
        """
        fx = self.depth_info.K[0]
        fy = self.depth_info.K[4]
        cx = self.depth_info.K[2]
        cy = self.depth_info.K[5]
        x, y, z = point_3d
        if z <= 0:
            return None
        u = int(round((x * fx) / z + cx))
        v = int(round((y * fy) / z + cy))
        return (u, v)

    def find_pointed_bag_pixel(self, wrist, elbow, bag_mask):
        """
        wrist, elbow: (u, v) tuples in pixel space
        bag_mask: binary (H, W) mask
        Returns: (u, v) intersection pixel, or None
        """
        u0, v0 = wrist
        u1, v1 = elbow
        du, dv = u0 - u1, v0 - v1
        norm = np.hypot(du, dv)
        if norm < 1e-4:
            return None
        du, dv = du / norm, dv / norm

        H, W = bag_mask.shape
        steps = int(2 * max(H, W))  # enough steps to cross image

        u, v = u0, v0
        for _ in range(steps):
            uu = int(round(u))
            vv = int(round(v))
            if 0 <= uu < W and 0 <= vv < H:
                if bag_mask[vv, uu]:
                    return (uu, vv)
            u += du
            v += dv
        return None

    def pixel_to_3d(self, u, v):
        # Read raw depth (uint16 mm) at (v, u)
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

    def get_human_keypoints_3d(self):
        if (
            self.latest_rgb is None
            or self.latest_depth is None
            or self.depth_info is None
        ):
            return None

        # Convert raw uint16 depth (mm) → float32 meters for the service
        depth_m = self.latest_depth.astype(np.float32) * 0.001
        depth_msg_f32 = self.bridge.cv2_to_imgmsg(depth_m, "32FC1")

        req = YoloPoseDetection3DRequest(
            image_raw=self.bridge.cv2_to_imgmsg(self.latest_rgb, "bgr8"),
            depth_image=depth_msg_f32,
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

        kp_list = res.detections[0].keypoints
        return {
            kp.keypoint_name: np.array(
                [kp.point.x / 1000.0, kp.point.y / 1000.0, kp.point.z / 1000.0]
            )
            for kp in kp_list
        }

    def get_bag_centroid_camera_frame(self):
        """
        1) Run SAM to get a bag_mask
        2) Compute the 2D centroid of bag_mask
        3) Back‐project that centroid pixel → 3D in camera frame (meters)
        Returns:
        - np.array([x_cam, y_cam, z_cam]) on success
        - None if any step fails
        """
        # Step 1: Run SAM → bag_mask (H×W uint8)
        bag_mask = self.get_bag_mask(self.latest_rgb)
        H, W = bag_mask.shape

        moments = cv2.moments(bag_mask)
        if moments["m00"] == 0:
            rospy.logwarn("[SAM] Bag mask is empty; no centroid.")
            return None

        # Step 2: 2D centroid (u,v) in pixel coordinates
        u_centroid = int(round(moments["m10"] / moments["m00"]))
        v_centroid = int(round(moments["m01"] / moments["m00"]))

        # Clamp to image bounds
        u_centroid = max(0, min(W - 1, u_centroid))
        v_centroid = max(0, min(H - 1, v_centroid))

        # Step 3: Read raw depth (uint16 mm) at (v_centroid, u_centroid)
        raw_depth_mm = float(self.latest_depth[v_centroid, u_centroid])
        if not np.isfinite(raw_depth_mm) or raw_depth_mm <= 0.0:
            rospy.logwarn("[SAM] Invalid depth at centroid pixel.")
            return None

        z_cam = raw_depth_mm * 0.001  # convert mm → meters
        if z_cam > 50.0:
            rospy.logwarn(f"[SAM] Depth too large after conversion: {z_cam:.2f}m")
            return None

        # Camera intrinsics from depth_info.K
        fx = self.depth_info.K[0]
        fy = self.depth_info.K[4]
        cx = self.depth_info.K[2]
        cy = self.depth_info.K[5]

        x_cam = (u_centroid - cx) * z_cam / fx
        y_cam = (v_centroid - cy) * z_cam / fy

        return np.array([x_cam, y_cam, z_cam])

    def get_bag_mask(self, rgb_bgr):
        """
        Run SAM to get a binary mask of “bag.” Take the union of all proposed masks
        and then pick the largest connected component.
        """
        img_rgb = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB)
        self.predictor.set_image(img_rgb)

        masks, _, _ = self.predictor.predict(
            point_coords=None,
            point_labels=None,
            multimask_output=True,
        )
        H, W = rgb_bgr.shape[:2]
        combined = np.zeros((H, W), dtype=np.uint8)
        for m in masks:
            combined = np.logical_or(combined, m).astype(np.uint8)

        # Keep only the largest connected component:
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            combined, connectivity=8
        )
        if num_labels <= 1:
            return np.zeros((H, W), dtype=np.uint8)

        areas = stats[1:, cv2.CC_STAT_AREA]
        largest_label = 1 + np.argmax(areas)
        bag_mask = (labels == largest_label).astype(np.uint8)
        return bag_mask

    def navigate_to_point_in_map(self, p_cam):
        """
        1) Transform p_cam ([x_cam, y_cam, z_cam] in camera frame) → p_map in 'map'
        2) Compute the robot’s goal XY 0.3 m behind the bag along camera→bag line
        3) Compute yaw so the robot faces the bag (from its goal pose)
        4) Send a MoveBaseGoal
        Returns True on success, False on any failure.
        """
        # Step 1: Transform from camera → map using TF
        cam_frame = self.depth_info.header.frame_id  # e.g. "xtion_depth_optical_frame"
        try:
            self.tf_listener.waitForTransform(
                "map", cam_frame, rospy.Time(0), rospy.Duration(1.0)
            )
            (trans, rot) = self.tf_listener.lookupTransform(
                "map", cam_frame, rospy.Time(0)
            )
        except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
            rospy.logerr(f"[SAM] TF lookup failed: {e}")
            return False

        # Build 4×4 transform matrix from rot+trans
        T = tf.transformations.quaternion_matrix(rot)
        T[0:3, 3] = trans
        p_cam_h = np.array([p_cam[0], p_cam[1], p_cam[2], 1.0])
        p_map_h = T.dot(p_cam_h)
        p_map = p_map_h[:3]  # [x_bag, y_bag, z_bag]

        # If the bag center is absurdly far, bail
        if np.linalg.norm(p_map[:2]) > 100.0:
            rospy.logwarn(f"[SAM] Mapped bag center too far: {p_map[:2]}")
            return False

        x_bag = p_map[0]
        y_bag = p_map[1]

        # Step 2: Compute goal position 0.3 m behind the bag along camera→bag line
        # First find camera position in map:
        try:
            self.tf_listener.waitForTransform(
                "map", cam_frame, rospy.Time(0), rospy.Duration(1.0)
            )
            (cam_trans, _) = self.tf_listener.lookupTransform(
                "map", cam_frame, rospy.Time(0)
            )
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logwarn(
                "[SAM] Could not lookup camera position; defaulting goal to bag location."
            )
            cam_trans = [x_bag, y_bag, 0.0]

        x_cam_map = cam_trans[0]
        y_cam_map = cam_trans[1]

        # Vector from bag → camera (in XY):
        dx = x_cam_map - x_bag
        dy = y_cam_map - y_bag
        norm = math.hypot(dx, dy)
        if norm < 1e-4:
            # If the camera is essentially on top of the bag in XY, choose an arbitrary offset direction
            dx, dy = 1.0, 0.0
            norm = 1.0

        # Unit vector from bag toward camera
        ux = dx / norm
        uy = dy / norm

        # Back off 0.3 m from the bag toward camera → goal_xy
        approach_dist = 0.3
        goal_x = x_bag - approach_dist * ux
        goal_y = y_bag - approach_dist * uy
        goal_z = 0.0  # keep goal on the floor

        # Step 3: Compute yaw so the robot faces the bag from the goal point
        # Vector from goal → bag (in XY) is (x_bag - goal_x, y_bag - goal_y):
        vx = x_bag - goal_x
        vy = y_bag - goal_y
        yaw = math.atan2(vy, vx)

        # Step 4: Build and send MoveBaseGoal
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
            f"[SAM‐CENTROID] Navigating to ({goal_x:.2f}, {goal_y:.2f}) facing bag at ({x_bag:.2f}, {y_bag:.2f}) yaw {yaw:.2f}"
        )
        self.move_base.send_goal(goal)
        ok = self.move_base.wait_for_result(rospy.Duration(30.0))
        if not ok or self.move_base.get_state() != actionlib.GoalStatus.SUCCEEDED:
            rospy.logerr("[SAM] Navigation to bag FAILED.")
            return False

        return True

    def _make_overlay(self, rgb_img: np.ndarray, mask: np.ndarray) -> np.ndarray:
        """
        Returns a BGR image where the mask is overlaid in semi‐transparent red.
        - rgb_img: H×W×3 uint8 (BGR)
        - mask:    H×W uint8 (0 or 1)
        """
        # Create a 3‐channel version of the mask in red
        red_mask = np.zeros_like(rgb_img, dtype=np.uint8)
        red_mask[:, :, 2] = (mask * 255).astype(np.uint8)

        alpha = 0.4  # transparency factor
        overlay = cv2.addWeighted(rgb_img, 1.0, red_mask, alpha, 0.0)
        return overlay


if __name__ == "__main__":
    try:
        PointAndNavigateNode()
    except rospy.ROSInterruptException:
        pass
