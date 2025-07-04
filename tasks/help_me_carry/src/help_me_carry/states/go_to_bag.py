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
from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction

from pal_interaction_msgs.msg import TtsAction, TtsGoal, TtsText


class GoToBag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        # --- Model and CV ---
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.depth_info = None
        self.latest_depth_msg = None
        self.latest_rgb_msg = None
        self.bag_prompt = "grocery bag"
        self.recovery_motions = [
            "look_down_centre",
            "look_down_left",
            "look_down_right",
        ]

        # Subscribers
        rgb_sub = Subscriber("/xtion/rgb/image_raw", Image)
        depth_sub = Subscriber("/xtion/depth_registered/image_raw", Image)
        info_sub = Subscriber("/xtion/depth_registered/camera_info", CameraInfo)
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

        self.motion_client = actionlib.SimpleActionClient(
            "/play_motion", PlayMotionAction
        )
        rospy.loginfo("Waiting for /play_motion action server...")
        self.motion_client.wait_for_server()
        rospy.loginfo("/play_motion action server connected")

        self.tts_client = actionlib.SimpleActionClient("tts", TtsAction)
        rospy.loginfo("Waiting for TTS action server…")
        self.tts_client.wait_for_server()
        rospy.loginfo("TTS action server connected.")

        rospy.sleep(3)

        cv2.namedWindow("Auto-segmented bag", cv2.WINDOW_NORMAL)

    def amcl_pose_callback(self, msg):
        self.latest_amcl_pose = msg

    def synced_callback(self, rgb_msg, depth_msg, info_msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(
                rgb_msg, desired_encoding="bgr8"
            )
            self.latest_rgb_msg = rgb_msg
            self.latest_depth_msg = depth_msg
            self.latest_depth = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough"
            )
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
            depth_float = self.latest_depth.astype(np.float32)
        else:
            # Assume it's already float32 in meters
            depth_float = self.latest_depth

        req = YoloPoseDetection3DRequest(
            image_raw=self.bridge.cv2_to_imgmsg(self.latest_rgb, encoding="bgr8"),
            depth_image=self.bridge.cv2_to_imgmsg(depth_float, encoding="32FC1"),
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
            kp.keypoint_name: np.array([kp.point.x, kp.point.y, kp.point.z])
            for kp in kp_list
        }

    def say(self, text: str):
        if hasattr(self, "tts_client"):
            goal = TtsGoal(rawtext=TtsText(text=text, lang_id="en_GB"))
            self.tts_client.send_goal(goal)
            self.tts_client.wait_for_result()
        else:
            rospy.loginfo(f"[TTS fallback] {text}")

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

        # ------ Look where the person points ------
        self.say("I'm looking at where you're pointing.")
        floor_pt = self.look_where_person_points()

        if floor_pt is None:
            # Get robot current pose, set z=0.6
            try:
                self.tf_listener.waitForTransform(
                    "map", "base_footprint", rospy.Time(0), rospy.Duration(1.0)
                )
                (trans, rot) = self.tf_listener.lookupTransform(
                    "map", "base_footprint", rospy.Time(0)
                )
                robot_point = np.array([trans[0], trans[1], 0.6])
            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                rospy.logwarn("Could not get robot position.")
                return "failed"
            floor_pt = robot_point

        self.say("I'm trying to find the bag.")
        centroid, mask, det = self.detect_bag_with_lang_sam(
            prompt=self.bag_prompt, floor_pt=floor_pt
        )

        recovery_counter = 0
        while centroid is None and recovery_counter < 3:
            rospy.logwarn(
                f"Recovery mode with head looking {self.recovery_motions[recovery_counter]}"
            )
            self.execute_play_motion(self.recovery_motions[recovery_counter])
            centroid, mask, det = self.detect_bag_with_lang_sam(
                prompt=self.bag_prompt, floor_pt=floor_pt
            )
            recovery_counter = recovery_counter + 1

        if centroid is None:
            rospy.logwarn("No bag detected in recovery mode!")
            return "failed"

        self.visualize_segmentation(self.latest_rgb, mask)
        p_cam = centroid

        head_point = self.transform_from_camera_to_map(p_cam)
        self.say("I'm navigating to the bag.")
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

    def look_where_person_points(self):
        rospy.loginfo("Looking for pointing gesture (by closest left ankle)...")
        kps_result = self.get_human_keypoints_3d()

        if not kps_result:
            rospy.logwarn("No human keypoints detected!")
            return None

        # Normalize to a list of dicts
        if isinstance(kps_result, dict):
            all_kps = [kps_result]
        elif isinstance(kps_result, list):
            all_kps = kps_result
        else:
            rospy.logwarn("Keypoints format not recognized!")
            return None

        if len(all_kps) == 0:
            rospy.logwarn("No keypoints detected at all!")
            return None

        if len(all_kps) == 1:
            # Only one person detected: just use them
            kps = all_kps[0]
        else:
            # Multiple people: pick the one with mean keypoint position closest to robot
            closest_kps = None
            min_dist = float("inf")
            for kps_candidate in all_kps:
                pts = np.array(
                    [pt for pt in kps_candidate.values() if not np.allclose(pt, 0)]
                )
                if pts.size == 0:
                    continue
                mean_pt = np.mean(pts, axis=0)
                dist = np.linalg.norm(mean_pt)
                if dist < min_dist:
                    min_dist = dist
                    closest_kps = kps_candidate
            if closest_kps is None:
                rospy.logwarn("No valid keypoints found for any person.")
                return None
            kps = closest_kps

        # Transform all keypoints to map frame
        kps_map = {}
        for name, p_cam in kps.items():
            p_map = self.transform_from_camera_to_map(p_cam)
            if p_map is not None:
                kps_map[name] = np.array(p_map)
            else:
                rospy.logwarn(f"Could not transform {name} to map frame")

        map_down_axis = np.array([0, 0, -1])
        arms = []

        for side in ["right", "left"]:
            elbow_key = f"{side}_elbow"
            wrist_key = f"{side}_wrist"
            if all(k in kps_map for k in [elbow_key, wrist_key]):
                origin = kps_map[elbow_key]
                wrist = kps_map[wrist_key]
                vec = wrist - origin
                norm = np.linalg.norm(vec)
                if norm < 1e-6:
                    continue
                dirv = vec / norm
                downward = np.dot(dirv, map_down_axis)
                rospy.loginfo(f"{side} arm downward={downward:.2f}")
                arms.append(
                    {"side": side, "downward": downward, "dirv": dirv, "origin": origin}
                )

        arms = sorted(arms, key=lambda x: x["downward"])

        for arm in arms:
            rospy.loginfo(f"Trying {arm['side']} arm for pointing ray...")
            floor_pt = self.find_pointed_floor(arm["origin"], arm["dirv"])
            if floor_pt is not None:
                floor_pt[2] = 0.65  # Force Z up from ground for natural gaze
                self.look_at_point(floor_pt, "map")
                rospy.loginfo(f"Looking at pointed floor position (map): {floor_pt}")
                rospy.sleep(1.0)
                return floor_pt
            else:
                rospy.logwarn(f"No valid floor intersection for {arm['side']} arm.")

        rospy.logwarn(
            "No valid pointing direction found (in map frame) with either arm."
        )
        return None

    def find_pointed_floor(self, origin, direction, max_dist=4.0):
        # direction: should be unit vector, both in map frame
        if abs(direction[2]) < 1e-6:
            rospy.logwarn(
                "Pointing direction is parallel to the floor, no intersection."
            )
            return None
        d = -origin[2] / direction[2]
        if d < 0 or d > max_dist:
            rospy.logwarn("Intersection behind or too far away.")
            return None
        elif d > max_dist:
            rospy.logwarn("Intersection behind or too far away.")
            return None
        hit_pt = origin + direction * d
        rospy.loginfo(f"Ray hits floor at: {hit_pt}")
        return hit_pt

    def detect_bag_with_lang_sam(self, floor_pt, prompt="bag"):
        """
        Calls the LangSAM segmentation service to detect a bag using the latest RGB image.
        Returns: (centroid_cam, mask, detection) for the bag closest to floor_pt, or (None, None, None) on failure.
        """
        if self.latest_rgb is None:
            rospy.logerr("No RGB image available for segmentation.")
            return None, None, None

        if self.latest_depth is None:
            rospy.logerr("No depth image available for segmentation.")
            return None, None, None

        rgb_msg = self.latest_rgb_msg
        depth_msg = self.latest_depth_msg
        langsam_req = LangSamRequest()
        langsam_req.image_raw = rgb_msg
        langsam_req.prompt = prompt
        langsam_req.depth_image = depth_msg
        langsam_req.depth_camera_info = self.depth_info
        langsam_req.box_threshold = 0.3
        langsam_req.text_threshold = 0.3
        langsam_req.target_frame = "xtion_rgb_optical_frame"

        try:
            resp = self.langsam_srv(langsam_req)
        except rospy.ServiceException as e:
            rospy.logerr(f"LangSAM service call failed: {e}")
            return None, None, None

        if not resp.detections:
            rospy.logwarn(f"No '{prompt}' detected by LangSAM.")
            cv2.destroyAllWindows()
            return None, None, None

        height, width = self.latest_rgb.shape[:2]
        closest_dist = float("inf")
        closest_centroid = None
        closest_mask = None
        closest_det = None

        print(len(resp.detections))
        for det in resp.detections:
            mask_flat = np.array(det.seg_mask, dtype=np.uint8)
            if mask_flat.size != height * width:
                rospy.logwarn(f"Skipping mask with wrong shape")
                continue
            mask = mask_flat.reshape((height, width))

            # Find median centroid in image space
            ys, xs = np.where(mask)
            if len(xs) == 0 or len(ys) == 0:
                continue
            u = int(np.median(xs))
            v = int(np.median(ys))

            # Convert to 3D (camera frame)
            p_cam = self.pixel_to_3d(u, v)
            if p_cam is None:
                continue

            # Transform to map frame for comparison
            centroid_map = self.transform_from_camera_to_map(p_cam)
            if centroid_map is None:
                continue

            dist = np.linalg.norm(np.array(centroid_map[:2]) - np.array(floor_pt[:2]))
            if dist < closest_dist:
                closest_dist = dist
                closest_centroid = p_cam
                closest_mask = mask
                closest_det = det

        if closest_centroid is None:
            rospy.logwarn("No valid bag centroid near the pointing floor point.")
            cv2.destroyAllWindows()
            return None, None, None

        return closest_centroid, closest_mask, closest_det

    def execute_play_motion(self, motion_name):
        goal = PlayMotionGoal()
        goal.motion_name = motion_name
        goal.skip_planning = False
        self.motion_client.send_goal(goal)
        self.motion_client.wait_for_result()
        rospy.loginfo(f"Motion {motion_name} played")

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
        z_cam = raw_depth_mm  # meters
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
        cam_frame = "xtion_rgb_optical_frame"

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
            f"Request to face ({target_x:.2f}, {target_y:.2f}) using AMCL pose."
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
                rospy.loginfo("Already facing target.")
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
