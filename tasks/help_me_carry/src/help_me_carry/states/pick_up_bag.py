#!/usr/bin/env python3.10

import rospy
import cv2
import numpy as np
import math
import copy
import actionlib
import tf
import tf.transformations as tft

from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import (
    MoveGroupCommander,
    roscpp_initialize,
    roscpp_shutdown,
    PlanningSceneInterface,
)
from message_filters import Subscriber, ApproximateTimeSynchronizer
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pal_interaction_msgs.msg import TtsAction, TtsGoal, TtsText
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningSceneRequest
from moveit_msgs.msg import AllowedCollisionEntry
from lasr_vision_msgs.srv import LangSam, LangSamRequest

import smach


def allow_collisions_with_object(obj_name, scene):
    """Updates the MoveIt PlanningScene using the AllowedCollisionMatrix to ignore collisions for an object"""
    # Set up service to get the current planning scene
    service_timeout = 5.0
    _get_planning_scene = rospy.ServiceProxy("get_planning_scene", GetPlanningScene)
    _get_planning_scene.wait_for_service(service_timeout)

    request = GetPlanningScene()
    request.components = 0  # Get just the Allowed Collision Matrix
    planning_scene = _get_planning_scene.call(request)

    # Set this object to ignore collisions with all objects. The entry values are not updated
    planning_scene.scene.allowed_collision_matrix.entry_names.append(obj_name)
    for entry in planning_scene.scene.allowed_collision_matrix.entry_values:
        entry.enabled.append(True)
    enabled = [
        True
        for i in range(len(planning_scene.scene.allowed_collision_matrix.entry_names))
        if planning_scene.scene.allowed_collision_matrix.entry_names[i]
        in ["gripper_left_finger_link", "gripper_right_finger_link", "gripper_link"]
    ]
    entry = AllowedCollisionEntry(enabled=enabled)  # Test kwarg in constructor
    planning_scene.scene.allowed_collision_matrix.entry_values.append(entry)

    # Set the default entries. They are also not updated
    planning_scene.scene.allowed_collision_matrix.default_entry_names = [obj_name]
    planning_scene.scene.allowed_collision_matrix.default_entry_values = [False]
    planning_scene.scene.is_diff = True  # Mark this as a diff message to force an update of the allowed collision matrix
    planning_scene.scene.robot_state.is_diff = True

    planning_scene_diff_req = ApplyPlanningSceneRequest()
    planning_scene_diff_req.scene = planning_scene.scene

    # Updating the Allowed Collision Matrix through the apply_planning_scene service shows no effect.
    # However, adding objects to the planning scene works fine.
    # scene._apply_planning_scene_diff.call(planning_scene_diff_req)
    scene.apply_planning_scene(planning_scene.scene)

    # Attempting to use the planning_scene topic for asynchronous updates also does not work
    # planning_scene_pub = rospy.Publisher("planning_scene", PlanningScene, queue_size=5)
    # planning_scene_pub.publish(planning_scene.scene)

    # The planning scene retrieved after the update should have taken place shows the Allowed Collision Matrix is the same as before
    request = GetPlanningScene()
    request.components = 0  # Get just the Allowed Collision Matrix
    planning_scene = _get_planning_scene.call(request)

    rospy.loginfo("Collision object added to disregard to the planning scene!")


class BagPickAndPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None

        # Display window for clicking
        cv2.namedWindow("Live View", cv2.WINDOW_NORMAL)

        # Sync RGB + depth
        rgb_sub = Subscriber("/xtion/rgb/image_raw", Image)
        depth_sub = Subscriber("/xtion/depth/image_raw", Image)
        ats = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.1)
        ats.registerCallback(self.synced_callback)

        # --- MoveIt! arm setup ---
        roscpp_initialize([])
        self.arm = MoveGroupCommander("arm_torso")

        # --- Gripper action client ---
        self.gripper_client = actionlib.SimpleActionClient(
            "/parallel_gripper_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        rospy.loginfo("Waiting for gripper action server…")
        self.gripper_client.wait_for_server()
        rospy.loginfo("Gripper action server ready.")

        self.motion_client = actionlib.SimpleActionClient(
            "/play_motion", PlayMotionAction
        )
        rospy.loginfo("Waiting for /play_motion action server...")
        self.motion_client.wait_for_server()
        rospy.loginfo("/play_motion action server connected")

        self.torso_client = actionlib.SimpleActionClient(
            "/torso_controller/follow_joint_trajectory", FollowJointTrajectoryAction
        )
        rospy.loginfo("Waiting for torso action server…")
        self.torso_client.wait_for_server()
        rospy.loginfo("Torso action server ready.")

        self.tts_client = actionlib.SimpleActionClient("tts", TtsAction)
        rospy.loginfo("Waiting for TTS action server…")
        self.tts_client.wait_for_server()
        rospy.loginfo("TTS action server connected.")

        self.planning_scene_interface = PlanningSceneInterface()
        self.planning_scene_interface.clear()

        # --- LangSAM client ---
        rospy.loginfo("Waiting for /lasr_vision/lang_sam service...")
        rospy.wait_for_service("/lasr_vision/lang_sam")
        self.langsam_srv = rospy.ServiceProxy("/lasr_vision/lang_sam", LangSam)
        rospy.loginfo("/lasr_vision/lang_sam service connected.")

        self.tf_listener = tf.TransformListener()

    def synced_callback(self, rgb_msg, depth_msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            self.latest_depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except Exception as e:
            rospy.logerr(f"cv_bridge error: {e}")

    def execute(self, userdata):
        try:
            self.adjust_torso(0.1)
            self.say("I am detecting the bag")
            self.wait_for_rgb()
            resp = self.get_langsam_bags()
            if not resp or not resp.detections:
                rospy.logwarn("No bags detected.")
                return "failed"

            intrinsics = self.get_camera_intrinsics()
            result = self.find_closest_detection(
                resp.detections,
                self.latest_rgb,
                self.latest_depth,
                intrinsics,
            )
            if result is None:
                rospy.logwarn("No valid bag detections found.")
                return "failed"
            mask, pts, centroid = result["mask"], result["pts"], result["centroid"]

            self.show_mask_overlay(mask)
            yaw = self.compute_yaw(pts, centroid)
            closest_pose = self.find_closest_pose_footprint(pts)
            centroid_pose = self.transform_centroid_pose_footprint(centroid, yaw)
            self.create_collision_object_from_pcl(pts)
            rospy.sleep(1.0)
            self.say("I am picking up the bag now.")
            self.prepare_pick()
            if not self.pick(centroid_pose, closest_pose):
                self.ask_for_bag()
            self.stow_bag()
            self.hold_gripper_position()
            return "succeeded"
        except Exception as e:
            rospy.logerr(f"Exception in skill: {e}")
            try:
                cv2.destroyAllWindows()
            except:
                pass
            return "failed"

    def wait_for_rgb(self):
        rate = rospy.Rate(5)
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.latest_rgb is not None:
                return
            rate.sleep()
            if (rospy.Time.now() - start).to_sec() > 10:
                raise RuntimeError("Timeout waiting for camera.")

    def get_langsam_bags(self):
        rgb_msg = self.bridge.cv2_to_imgmsg(self.latest_rgb, "bgr8")
        req = LangSamRequest()
        req.image_raw = rgb_msg
        req.prompt = "carrier bag"
        try:
            return self.langsam_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr(f"LangSAM service call failed: {e}")
            return None

    def get_camera_intrinsics(self):
        # Ideally, fetch from CameraInfo topic!
        return dict(fx=579.653076171875, fy=579.653076171875, cx=319.5, cy=239.5)

    def mask_to_3d_points(self, mask, depth, fx, fy, cx, cy):
        yy, xx = np.where(mask)
        pts = []
        for y, x in zip(yy, xx):
            z = depth[y, x]
            if np.isfinite(z) and z > 0.05:
                X = (x - cx) * z / fx
                Y = (y - cy) * z / fy
                pts.append((X, Y, z))
        if pts:
            return np.array(pts)
        else:
            return None

    def transform_point_to_base(self, point_xyz):
        listener = self.tf_listener  # Instantiate tf_listener ONCE in __init__!
        listener.waitForTransform(
            "base_footprint",
            "xtion_rgb_optical_frame",
            rospy.Time(0),
            rospy.Duration(1.0),
        )
        trans, rot = listener.lookupTransform(
            "base_footprint", "xtion_rgb_optical_frame", rospy.Time(0)
        )
        T = tft.quaternion_matrix(rot)
        T[0:3, 3] = trans
        hom = np.hstack([point_xyz / 1000.0, 1.0])
        pt_base = T @ hom
        return pt_base[:3]

    def find_closest_detection(self, detections, rgb, depth, intrinsics):
        height, width = rgb.shape[:2]
        min_dist = float("inf")
        best = None
        for det in detections:
            mask_flat = np.array(det.seg_mask, dtype=np.uint8)
            if mask_flat.size != height * width:
                continue
            mask = mask_flat.reshape((height, width))
            pts = self.mask_to_3d_points(mask, depth, **intrinsics)
            if pts is None:
                continue
            centroid = np.median(pts, axis=0)
            try:
                centroid_base = self.transform_point_to_base(centroid)
                dist = np.linalg.norm(centroid_base[:2])
            except Exception as e:
                rospy.logwarn(f"TF error: {e}")
                continue
            if dist < min_dist:
                min_dist = dist
                best = dict(det=det, mask=mask, pts=pts, centroid=centroid)
        return best

    def show_mask_overlay(self, mask):
        overlay = self.latest_rgb.copy()
        overlay[mask > 0] = [0, 0, 255]
        cv2.imshow("Auto-segmented bag", overlay)
        cv2.waitKey(1000)
        cv2.destroyWindow("Auto-segmented bag")

    def compute_yaw(self, pts, centroid):
        xy = pts[:, :2] - centroid[:2]
        cov = np.cov(xy, rowvar=False)
        evals, evecs = np.linalg.eigh(cov)
        long_axis = evecs[:, np.argmax(evals)]
        orth = np.array([-long_axis[1], long_axis[0]])
        orth /= np.linalg.norm(orth)
        yaw = math.atan2(orth[1], orth[0])
        return yaw

    def create_collision_object_from_pcl(self, points):
        min_pt = points.min(axis=0)
        max_pt = points.max(axis=0)

        center = (min_pt + max_pt) / 2.0
        size = (max_pt - min_pt) / 1000.0

        # --- PCA on XY as you do elsewhere
        xy = points[:, :2] - center[:2]
        cov = np.cov(xy, rowvar=False)
        evals, evecs = np.linalg.eigh(cov)
        # The principal axis in the XY plane
        long_axis = evecs[:, np.argmax(evals)]
        orth = np.array([-long_axis[1], long_axis[0]])  # orthogonal

        # Build rotation matrix
        x_axis = np.array([long_axis[0], long_axis[1], 0])
        y_axis = np.array([orth[0], orth[1], 0])
        z_axis = np.array([0, 0, 1])
        R = np.stack([x_axis, y_axis, z_axis], axis=1)  # shape (3,3)

        quat = tft.quaternion_from_matrix(
            np.vstack([np.hstack([R, np.zeros((3, 1))]), [0, 0, 0, 1]])  # shape (3,4)
        )

        p = PoseStamped()
        frame_id = "xtion_rgb_optical_frame"
        p.header.frame_id = frame_id
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = center[0] / 1000.0
        p.pose.position.y = center[1] / 1000.0
        p.pose.position.z = center[2] / 1000.0
        p.pose.orientation.x = quat[0]
        p.pose.orientation.y = quat[1]
        p.pose.orientation.z = quat[2]
        p.pose.orientation.w = quat[3]

        rospy.loginfo(p)
        self.planning_scene_interface.add_box("obj", p, size=size)

        allow_collisions_with_object("obj", self.planning_scene_interface)
        rospy.loginfo("Collision object added to the planning scene!")

    def adjust_torso(self, target_height):
        rospy.loginfo("Adjusting torso")
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["torso_lift_joint"]

        point = JointTrajectoryPoint()
        point.positions = [target_height]
        point.time_from_start = rospy.Duration(2.0)
        goal.trajectory.points.append(point)
        self.torso_client.send_goal(goal)
        self.torso_client.wait_for_result()

    def prepare_pick(self):
        rospy.loginfo("Preparing to pick up the bag…")
        self.open_gripper()
        goal = PlayMotionGoal()
        goal.motion_name = "prepare_grasp"
        goal.skip_planning = False
        self.motion_client.send_goal(goal)
        self.motion_client.wait_for_result()
        self.adjust_torso(0.1)
        rospy.sleep(0.2)
        rospy.loginfo("Reached prepare grasp pose.")

    def find_closest_pose_footprint(self, pts):
        # Transform all pts to base_footprint
        listener = tf.TransformListener()
        listener.waitForTransform(
            "base_footprint",
            "xtion_rgb_optical_frame",
            rospy.Time(0),
            rospy.Duration(1.0),
        )
        trans, rot = listener.lookupTransform(
            "base_footprint", "xtion_rgb_optical_frame", rospy.Time(0)
        )
        T = tft.quaternion_matrix(rot)
        T[0:3, 3] = trans

        pts_m = pts / 1000.0  # if needed
        pts_hom = np.hstack([pts_m, np.ones((pts_m.shape[0], 1))])
        pts_base = (T @ pts_hom.T).T[:, :3]

        dists = np.linalg.norm(pts_base, axis=1)
        k = 10
        closest_index = np.argmin(dists)
        closest_pts = pts_base[closest_index]  # still in camera frame

        pose = PoseStamped()
        pose.header.frame_id = "base_footprint"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = closest_pts[0]
        pose.pose.position.y = closest_pts[1]
        pose.pose.position.z = closest_pts[
            2
        ]  # Top point is twice the height of the centroid

        pose.pose.orientation.x = 0.0010062689510449319
        pose.pose.orientation.y = 0.70710678
        pose.pose.orientation.z = 0.0004933099788376551
        pose.pose.orientation.w = 0.70710678

        if pose.pose.position.z < 0.6:
            pose.pose.position.z = 0.6

        rospy.loginfo(f"Closest pick pose in base_footprint: {pose}")
        return pose

    def transform_centroid_pose_footprint(self, centroid_mm, yaw):
        x_mm, y_mm, z_mm = centroid_mm
        x_cam, y_cam, z_cam = x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0

        listener = tf.TransformListener()
        listener.waitForTransform(
            "base_footprint",
            "xtion_rgb_optical_frame",
            rospy.Time(0),
            rospy.Duration(1.0),
        )
        trans, rot = listener.lookupTransform(
            "base_footprint", "xtion_rgb_optical_frame", rospy.Time(0)
        )
        T = tft.quaternion_matrix(rot)
        T[0:3, 3] = trans
        x, y, z = T.dot([x_cam, y_cam, z_cam, 1.0])[:3]

        close_cam = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        R_cam2base = tft.quaternion_matrix(rot)[0:3, 0:3]
        close_base = R_cam2base.dot(close_cam)
        close_base[2] = 0.0
        close_base /= np.linalg.norm(close_base)
        yaw_base = math.atan2(close_base[1], close_base[0])
        q_final = tft.quaternion_from_euler(0, math.pi / 2, yaw_base)

        pose = PoseStamped()
        pose.header.frame_id = "base_footprint"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z * 2  # Top point is twice the height of the centroid

        pose.pose.orientation.x = float(q_final[0])
        pose.pose.orientation.y = float(q_final[1])
        pose.pose.orientation.z = float(q_final[2])
        pose.pose.orientation.w = float(q_final[3])

        if pose.pose.position.z < 0.6:
            pose.pose.position.z = 0.6

        rospy.loginfo(f"Centroid pick pose in base_footprint: {pose}")
        return pose

    def pick(self, centroid_pose, closest_pose):
        closest_pose.pose.orientation = centroid_pose.pose.orientation

        self.arm.set_start_state_to_current_state()
        pre_pose = copy.deepcopy(centroid_pose)
        pre_pose.pose.position.z += 0.20
        self.arm.set_pose_target(pre_pose)
        result = self.arm.go(wait=True)
        self.arm.clear_pose_targets()
        rospy.sleep(0.2)

        if result == False:
            rospy.logwarn(f"Picking with closest pose")
            self.arm.set_start_state_to_current_state()
            pre_pose = copy.deepcopy(closest_pose)
            pre_pose.pose.position.z += 0.20
            self.arm.set_pose_target(pre_pose)
            result = self.arm.go(wait=True)
            self.arm.clear_pose_targets()
            rospy.sleep(0.2)

        if result == False:
            return result

        result = self.sync_shift_ee(self.arm, 0.30, 0.0, 0.0)
        if result == False:
            return result
        self.close_gripper()
        rospy.loginfo("Bag pick complete!")
        self.sync_shift_ee(self.arm, -0.30, 0.0, 0.0)

        return self.is_picked_up(0.002, 0.10)

    def sync_shift_ee(self, move_group, x, y, z):
        from tf.transformations import euler_from_quaternion, euler_matrix

        pose = self.get_eef_pose_listener(
            ee_frame="arm_tool_link", base_frame="base_footprint"
        )
        quat = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        euler = euler_from_quaternion(quat)
        euler_m = euler_matrix(*euler)
        delta = np.dot(euler_m[:3, :3], np.array([x, y, z]).T)
        pose.position.x += delta[0]
        pose.position.y += delta[1]
        pose.position.z += delta[2]
        move_group.set_pose_target(pose)
        move_group.set_start_state_to_current_state()
        result = move_group.go(wait=True)
        return result

    def get_eef_pose_listener(
        self, ee_frame="arm_tool_link", base_frame="base_footprint"
    ):
        listener = tf.TransformListener()
        try:
            listener.waitForTransform(
                base_frame, ee_frame, rospy.Time(0), rospy.Duration(1.0)
            )
            trans, rot = listener.lookupTransform(base_frame, ee_frame, rospy.Time(0))

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = trans
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ) = rot
            return pose

        except Exception as e:
            rospy.logerr(
                f"Failed to get transform from {base_frame} to {ee_frame}: {e}"
            )
            return None

    def is_picked_up(self, pos_thresh=0.002, effort_thresh=0.05):
        self.close_gripper()
        rospy.sleep(0.2)
        js = rospy.wait_for_message("/joint_states", JointState, timeout=1.0)
        lidx = js.name.index("gripper_left_finger_joint")
        ridx = js.name.index("gripper_right_finger_joint")
        pos_l, pos_r = js.position[lidx], js.position[ridx]
        eff_l, eff_r = js.effort[lidx], js.effort[ridx]
        avg_pos = 0.5 * (pos_l + pos_r)
        avg_eff = abs(0.5 * (eff_l + eff_r))
        if avg_pos >= pos_thresh or avg_eff >= effort_thresh:
            rospy.loginfo(f"Grasp succeeded: pos={avg_pos:.3f} m, eff={avg_eff:.2f}")
            return True
        else:
            rospy.logwarn(f"No object: pos={avg_pos:.3f} m, eff={avg_eff:.2f}")
            return False

    def ask_for_bag(self):
        self.open_gripper()
        goal = PlayMotionGoal()
        goal.motion_name = "offer_gripper"
        goal.skip_planning = False
        self.motion_client.send_goal(goal)
        self.motion_client.wait_for_result()
        rospy.loginfo("Gripper offered.")
        self.say(
            "I wasn't able to pickup the bag. Please put the bag into my gripper. I will give you 5 seconds."
        )
        self.say("5")
        rospy.sleep(0.5)
        self.say("4")
        rospy.sleep(0.5)
        self.say("3")
        rospy.sleep(0.5)
        self.say("2")
        rospy.sleep(0.5)
        self.say("1")
        rospy.sleep(0.5)
        self.close_gripper()

    def say(self, text: str):
        if hasattr(self, "tts_client"):
            goal = TtsGoal(rawtext=TtsText(text=text, lang_id="en_GB"))
            self.tts_client.send_goal(goal)
            self.tts_client.wait_for_result()
        else:
            rospy.loginfo(f"[TTS fallback] {text}")

    def stow_bag(self):
        self.adjust_torso(0.3)
        goal = PlayMotionGoal()
        goal.motion_name = "stow_bag"
        goal.skip_planning = False
        self.motion_client.send_goal(goal)
        self.motion_client.wait_for_result()
        rospy.loginfo("Reached stow pose.")

    def close_gripper(self, width=0.00, duration=1.0):
        goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = ["gripper_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [width]
        pt.time_from_start = rospy.Duration(duration)
        traj.points = [pt]
        goal.trajectory = traj
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()

    def open_gripper(self, width=0.10, duration=1.0):
        goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = ["gripper_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [width]
        pt.time_from_start = rospy.Duration(duration)
        traj.points = [pt]
        goal.trajectory = traj
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()

    def hold_gripper_position(self, duration=1.0):
        js = rospy.wait_for_message("/joint_states", JointState, timeout=1.0)

        lidx = js.name.index("gripper_left_finger_joint")
        ridx = js.name.index("gripper_right_finger_joint")
        pos_l = js.position[lidx]
        pos_r = js.position[ridx]

        goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [pos_l, pos_r]
        pt.time_from_start = rospy.Duration(duration)
        traj.points = [pt]
        goal.trajectory = traj

        self.gripper_client.send_goal(goal)


if __name__ == "__main__":
    rospy.init_node("bag_pick_up")
    sm = smach.StateMachine(outcomes=["succeeded", "failed"])
    with sm:
        smach.StateMachine.add(
            "BAG_PICK_AND_PLACE",
            BagPickAndPlace(),
            transitions={"succeeded": "succeeded", "failed": "failed"},
        )
    outcome = sm.execute()
    rospy.loginfo(f"Skill outcome: {outcome}")
    roscpp_shutdown()
