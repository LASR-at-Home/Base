#!/usr/bin/env python3.10

import rospy
import cv2
import torch
import numpy as np
import os
import actionlib
import tf
import tf.transformations as tft

import copy
import math

import smach

from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from segment_anything import sam_model_registry, SamPredictor
from moveit_msgs import *
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
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3Stamped
from std_msgs.msg import Int64, Header
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningSceneRequest
from moveit_msgs.msg import (
    PlanningScene,
    AllowedCollisionEntry,
    Grasp,
    GripperTranslation,
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def allow_collisions_with_object(obj_name, scene):
    """Updates the MoveIt PlanningScene using the AllowedCollisionMatrix to ignore collisions for an object"""
    # Set up service to get the current planning scene
    service_timeout = 5.0
    _get_planning_scene = rospy.ServiceProxy("get_planning_scene", GetPlanningScene)
    _get_planning_scene.wait_for_service(service_timeout)

    request = GetPlanningScene()
    request.components = 0  # Get just the Allowed Collision Matrix
    planning_scene = _get_planning_scene.call(request)
    print(
        f"\n\n\n--- allowed_collision_matrix before update:{planning_scene.scene.allowed_collision_matrix} ---\n\n\n"
    )

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

    print(f"\n--- Sent message:{planning_scene.scene.allowed_collision_matrix} ---\n")

    # The planning scene retrieved after the update should have taken place shows the Allowed Collision Matrix is the same as before
    request = GetPlanningScene()
    request.components = 0  # Get just the Allowed Collision Matrix
    planning_scene = _get_planning_scene.call(request)
    print(
        f"\n--- allowed_collision_matrix after update:{planning_scene.scene.allowed_collision_matrix} ---\n"
    )


class BagPickAndPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

        # --- SAM2 setup ---
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        root = os.path.dirname(__file__)
        pkg = os.path.dirname(root)
        ckpt = os.path.join(pkg, "models", "sam_vit_b_01ec64.pth")
        sam = sam_model_registry["vit_b"](checkpoint=ckpt)
        sam.to(self.device)
        self.predictor = SamPredictor(sam)
        # -------------------

        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.click = None

        # Display window for clicking
        cv2.namedWindow("Live View", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Live View", self.on_mouse_click)

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

    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click = (x, y)
            rospy.loginfo(f"User click at {self.click}")

    def synced_callback(self, rgb_msg, depth_msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            self.latest_depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except Exception as e:
            rospy.logerr(f"cv_bridge error: {e}")

    def execute(self, userdata):
        try:
            self.adjust_torso(0.1)
            rate = rospy.Rate(30)
            start = rospy.Time.now()

            # Wait for RGB image
            while not rospy.is_shutdown():
                if self.latest_rgb is not None:
                    break
                rate.sleep()
                if (rospy.Time.now() - start).to_sec() > 10:  # 10s timeout
                    rospy.logwarn("Timeout waiting for camera.")
                    cv2.destroyWindow("Live View")
                    return "failed"

            frame = self.latest_rgb.copy()
            cv2.imshow("Live View", frame)
            cv2.waitKey(1)
            self.click = None

            # Wait for click, with a timeout
            click_start = rospy.Time.now()
            while not rospy.is_shutdown() and self.click is None:
                cv2.imshow("Live View", frame)
                cv2.waitKey(1)
                rate.sleep()
                if (rospy.Time.now() - click_start).to_sec() > 15:  # 15s timeout
                    rospy.logwarn("Timeout waiting for click.")
                    cv2.destroyWindow("Live View")
                    return "failed"

            if not self.click:
                rospy.logwarn("No click received.")
                cv2.destroyWindow("Live View")
                return "failed"

            u, v = self.click
            self.click = None

            # 1) segment with SAM
            self.predictor.set_image(frame)
            coords = np.array([[u, v]], dtype=float)
            labs = np.array([1], dtype=int)
            masks, _, _ = self.predictor.predict(
                point_coords=coords, point_labels=labs, multimask_output=False
            )
            mask = masks[0].astype(bool)

            pts = []
            h, w = mask.shape
            fx = fy = 579.653076171875  # verify with camera_info!
            cx, cy = 319.5, 239.5
            for yy, xx in zip(*np.where(mask)):
                z = self.latest_depth[yy, xx]
                if np.isfinite(z) and z > 0.05:
                    X = (xx - cx) * z / fx
                    Y = (yy - cy) * z / fy
                    pts.append((X, Y, z))
            if not pts:
                rospy.logwarn("No valid 3D points—try clicking elsewhere.")
                cv2.destroyWindow("Live View")
                return "failed"
            pts = np.array(pts)
            centroid = np.median(pts, axis=0)
            rospy.loginfo(f"Bag centroid (m): {centroid}")
            xy = pts[:, :2] - centroid[:2]
            cov = np.cov(xy, rowvar=False)
            evals, evecs = np.linalg.eigh(cov)
            long_axis = evecs[:, np.argmax(evals)]
            orth = np.array([-long_axis[1], long_axis[0]])
            orth /= np.linalg.norm(orth)
            yaw = math.atan2(orth[1], orth[0])

            closest_pose = self.find_closest_pose_footprint(pts)
            centroid_pose = self.transform_centroid_pose_footprint(centroid, yaw)

            # self.create_collision_object_from_pcl(pts)
            rospy.sleep(1.0)
            self.prepare_pick()
            result = self.pick(centroid_pose, closest_pose)
            if not result:
                self.ask_for_bag()

            self.stow_bag()
            cv2.destroyWindow("Live View")
            return "succeeded"
        except Exception as e:
            rospy.logerr(f"Exception in skill: {e}")
            try:
                cv2.destroyWindow("Live View")
            except:
                pass
            return "failed"

    def create_collision_object_from_pcl(self, points):
        # Optional: remove outliers based on percentiles (e.g., filtering out the extreme points)
        # lower = np.percentile(points, 1, axis=0)
        # upper = np.percentile(points, 99, axis=0)

        # Filter points based on the percentile ranges
        # mask = np.all((points >= lower) & (points <= upper), axis=1)
        # points = points[mask]

        # Find the min and max of the points
        min_pt = points.min(axis=0)
        max_pt = points.max(axis=0)

        # Calculate the center of the bounding box and its size
        center = (min_pt + max_pt) / 2.0
        size = (max_pt - min_pt) / 1000.0

        p = PoseStamped()
        frame_id = "xtion_rgb_optical_frame"
        p.header.frame_id = frame_id
        p.header.stamp = rospy.Time.now()
        # p.pose.position.x = obj.centroid.x
        # p.pose.position.y = obj.centroid.y
        # p.pose.position.z = obj.centroid.z
        p.pose.position.x = center[0] / 1000.0
        p.pose.position.y = center[1] / 1000.0
        p.pose.position.z = center[2] / 1000.0

        p.pose.orientation.w = 1
        rospy.loginfo(p)
        self.planning_scene_interface.add_box("obj", p, size=size)

        # # Create the collision object (bounding box)
        # collision_object = CollisionObject()
        # collision_object.header.frame_id = frame_id
        # collision_object.id = "object"

        # # Define a SolidPrimitive (box)
        # box = SolidPrimitive()
        # box.type = SolidPrimitive.BOX
        # box.dimensions = [size[0], size[1], size[2]]

        # # Define the pose of the collision object
        # pose = Pose()
        # pose.position.x = center[0]
        # pose.position.y = center[1]
        # pose.position.z = center[2]
        # pose.orientation.w = 1.0  # No rotation, just the position

        # collision_object.primitives = [box]
        # collision_object.primitive_poses = [pose]
        # collision_object.operation = CollisionObject.ADD

        # Add the collision object to the planning scene
        # planning_scene_interface.apply_collision_object(collision_object)
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

        result = self.sync_shift_ee(self.arm, 0.35, 0.0, 0.0)
        if result == False:
            return result
        self.close_gripper()
        rospy.loginfo("Bag pick complete!")
        self.sync_shift_ee(self.arm, -0.35, 0.0, 0.0)

        return self.is_picked_up(0.003, 0.20)

    def sync_shift_ee(self, move_group, x, y, z):
        from tf.transformations import euler_from_quaternion, euler_matrix

        curr_pose = move_group.get_current_pose()
        pose = curr_pose.pose
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
        move_group.set_pose_target(curr_pose)
        move_group.set_start_state_to_current_state()
        result = move_group.go(wait=True)
        return result

    def is_picked_up(self, pos_thresh=0.002, effort_thresh=0.05):
        rospy.sleep(0.1)
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
        rospy.sleep(5)
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


if __name__ == "__main__":
    import smach

    rospy.init_node("bag_pick_and_place_skill_runner")
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
