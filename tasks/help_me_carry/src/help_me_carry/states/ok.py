#!/usr/bin/env python3.10

import smach
import rospy
import numpy as np
import cv2
import actionlib
import tf
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from message_filters import Subscriber, ApproximateTimeSynchronizer

from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import PointStamped, Point

from lasr_vision_msgs.srv import LangSamRequest
from lasr_vision_msgs.srv import LangSam

from lasr_vision_msgs.srv import YoloPoseDetection3D, YoloPoseDetection3DRequest


class GoToBag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        # --- Model and CV ---
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.depth_info = None

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

        rospy.wait_for_service("/yolo/detect3d_pose")
        self.detect3d = rospy.ServiceProxy("/yolo/detect3d_pose", YoloPoseDetection3D)

        self.last_pointing_origin = None
        self.last_pointing_direction = None

        rospy.sleep(5)

        cv2.namedWindow("Auto-segmented bag", cv2.WINDOW_NORMAL)

    def amcl_pose_callback(self, msg):
        self.latest_amcl_pose = msg

    def synced_callback(self, rgb_msg, depth_msg, info_msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            self.latest_depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
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

        # Check if latest_depth is uint16 (16UC1)
        if self.latest_depth.dtype == np.uint16:
            # Convert from mm to meters, cast to float32
            depth_float = (self.latest_depth.astype(np.float32)) / 1000.0
        else:
            # Assume it's already float32 in meters
            depth_float = self.latest_depth

        req = YoloPoseDetection3DRequest(
            image_raw=self.bridge.cv2_to_imgmsg(self.latest_rgb, "bgr8"),
            depth_image=self.bridge.cv2_to_imgmsg(depth_float, "32FC1"),
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

        # ------ NEW: Look where the person points ------
        self.look_where_person_points()

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

        self.visualize_segmentation(self.latest_rgb, mask)

        ys, xs = np.where(mask)
        pts = []
        for v, u in zip(ys, xs):
            raw_depth_mm = float(self.latest_depth[v, u])
            if np.isfinite(raw_depth_mm) and raw_depth_mm > 0.0:
                z_cam = raw_depth_mm * 0.001
                fx = self.depth_info.K[0]
                fy = self.depth_info.K[4]
                cx = self.depth_info.K[2]
                cy = self.depth_info.K[5]
                x_cam = (u - cx) * z_cam / fx
                y_cam = (v - cy) * z_cam / fy
                pts.append([x_cam, y_cam, z_cam])
        if not pts:
            rospy.logwarn("No valid depth points in mask.")
            cv2.destroyAllWindows()
            return "failed"

        pts = np.array(pts)
        centroid = np.median(pts, axis=0)
        p_cam = centroid

        head_point = self.transform_from_camera_to_map(p_cam)
        x_bag, y_bag = self.navigate_to_closest_feasible_point(p_cam)
        if x_bag is not None and y_bag is not None:
            rospy.loginfo(
                "[LangSAM] Navigation succeeded! Adjusting orientation with AMCL pose."
            )
            self.face_point_with_amcl(x_bag, y_bag)
            self.look_at_point(head_point, "map")
            rospy.sleep(0.2)
            cv2.destroyAllWindows()
            return "succeeded"
        else:
            rospy.sleep(0.2)
            cv2.destroyAllWindows()
            return "failed"

    def look_where_person_points(self, max_head_moves=8, angle_step_deg=15.0):
        rospy.loginfo("Looking for pointing gesture...")

        # Initial head angle
        current_pan = 0.0
        for attempt in range(max_head_moves):
            # Try to detect human and pointing direction
            kps = self.get_human_keypoints_3d()
            new_gesture_found = False

            if kps:
                camera_z_axis = np.array([0.0, 0.0, 1.0])
                best_score = -1.0
                for side in ["right", "left"]:
                    elbow_key = f"{side}_elbow"
                    wrist_key = f"{side}_wrist"
                    if elbow_key in kps and wrist_key in kps:
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
                            self.last_pointing_origin = o
                            self.last_pointing_direction = dirv
                            new_gesture_found = True
                if new_gesture_found:
                    rospy.loginfo("Updated cached pointing gesture.")

            # Use cached pointing if we have it
            if (
                self.last_pointing_origin is not None
                and self.last_pointing_direction is not None
            ):
                o = self.last_pointing_origin
                dirv = self.last_pointing_direction
                floor_pt = self.find_pointed_floor(o, dirv)
                if isinstance(floor_pt, np.ndarray):
                    # Success: in frame!
                    head_point = self.transform_from_camera_to_map(floor_pt)
                    if head_point is not None:
                        self.look_at_point(head_point, "map")
                        rospy.loginfo(
                            f"Looking at pointed floor position: {head_point}"
                        )
                        rospy.sleep(1.0)
                        return True
                elif isinstance(floor_pt, str):
                    # Out of frame, rotate head
                    direction = floor_pt
                    rospy.loginfo(
                        f"Pointed direction is out of frame: {direction}. Rotating head."
                    )
                    self.rotate_head(direction, angle_step_deg)
                    rospy.sleep(0.7)  # Wait for head to move and image to update
                    continue
                else:
                    rospy.logwarn("Ray did not hit the floor or left frame.")
            else:
                rospy.logwarn("No valid pointing direction cached or detected!")
                rospy.sleep(0.7)
                continue

        rospy.logwarn("Max attempts reached. Could not find pointing intersection.")
        return False

    def find_pointed_floor(self, origin, direction, max_dist=4.0, step=0.02):
        for d in np.arange(0, max_dist, step):
            pt = origin - direction * d
            if pt[2] <= 0:
                if d == 0:
                    print(f"Ray starts below floor at: {pt}")
                    return pt
                prev_pt = origin + direction * (d - step)
                alpha = prev_pt[2] / (prev_pt[2] - pt[2])
                hit_pt = prev_pt + alpha * (pt - prev_pt)
                print(f"Ray hits floor at: {hit_pt}")
                return hit_pt
        print(f"Ray never hits floor for origin={origin}, direction={direction}")
        return None

    def rotate_head(self, direction, angle_step_deg=15.0):
        # This assumes you can control head pan via PointHeadAction.
        # You might want to keep track of head pan angle as a class variable if needed.
        # This simple version just sends a new pan in the given direction.
        axis_map = {
            "left": +1,
            "right": -1,
            "up": -1,  # for pitch, negative is up if using ROS standard (check your robot!)
            "down": +1,
        }
        angle_rad = np.deg2rad(angle_step_deg)
        # We'll rotate in pan only for left/right, pitch for up/down.
        # For a real robot, you may need to track limits!
        pan = 0.0
        tilt = 0.0

        if direction in ["left", "right"]:
            pan = axis_map[direction] * angle_rad
        elif direction in ["up", "down"]:
            tilt = axis_map[direction] * angle_rad

        # Compose target point a bit out in front in the new direction
        # (here, z=1.5m ahead; pan/tilt in camera frame)
        R_pan = np.array(
            [
                [np.cos(pan), 0, np.sin(pan)],
                [0, 1, 0],
                [-np.sin(pan), 0, np.cos(pan)],
            ]
        )
        R_tilt = np.array(
            [
                [1, 0, 0],
                [0, np.cos(tilt), -np.sin(tilt)],
                [0, np.sin(tilt), np.cos(tilt)],
            ]
        )
        # Combined rotation
        R = R_pan @ R_tilt
        look_vec = np.dot(R, np.array([0, 0, 1.5]))
        pt = Point()
        pt.x = float(look_vec[0])
        pt.y = float(look_vec[1])
        pt.z = float(look_vec[2])
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time(0)
        goal.target.header.frame_id = "xtion_rgb_optical_frame"
        goal.target.point = pt
        goal.pointing_frame = "xtion_rgb_optical_frame"
        goal.pointing_axis.x = 0.0
        goal.pointing_axis.y = 0.0
        goal.pointing_axis.z = 1.0
        goal.min_duration = rospy.Duration(0.2)
        goal.max_velocity = 0.6
        self.point_head_client.send_goal(goal)
        rospy.loginfo(
            f"Rotating head: pan {np.rad2deg(pan):.1f}°, tilt {np.rad2deg(tilt):.1f}°"
        )

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

        # Try moving to increasing distances back from the bag, up to 1m away
        approach_min = 0.1  # minimum approach distance (meters)
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
                f"Trying to navigate to ({goal_x:.2f}, {goal_y:.2f}) dist {approach_dist:.2f} from bag"
            )
            self.move_base.send_goal(goal)
            ok = self.move_base.wait_for_result()
            if ok and self.move_base.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Navigation succeeded at closest feasible point!")
                return (
                    x_bag,
                    y_bag,
                )  # return the true target for use in face_point_with_amcl
            else:
                rospy.logwarn(
                    "Navigation failed at this distance, trying further away..."
                )

        rospy.logerr("Could not find any feasible approach point.")
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

            rospy.loginfo(f"Rotating in place to yaw {desired_yaw:.2f} rad")
            self.move_base.send_goal(goal)
            ok = self.move_base.wait_for_result(rospy.Duration(6.0))
            if ok and self.move_base.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Rotation succeeded!")
                return True
            else:
                rospy.logwarn("Rotation failed, retrying...")
                rospy.sleep(0.5)
        rospy.logerr("Failed to adjust orientation after multiple tries.")
        return False

    def visualize_segmentation(self, rgb_img, mask):
        overlay = rgb_img.copy()
        overlay[mask > 0] = [0, 0, 255]
        print("RGB min/max", overlay.min(), overlay.max())
        print("Mask sum", mask.sum())
        cv2.imshow("Auto-segmented bag", overlay)
        cv2.waitKey(1000)

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

    rospy.init_node("go_to_bag")
    sm = smach.StateMachine(outcomes=["succeeded", "failed"])
    with sm:
        smach.StateMachine.add(
            "GO_TO_BAG",
            GoToBag(),
            transitions={"succeeded": "succeeded", "failed": "failed"},
        )
    outcome = sm.execute()
    rospy.loginfo(f"Skill outcome: {outcome}")
