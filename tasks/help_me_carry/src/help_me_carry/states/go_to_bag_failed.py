#!/usr/bin/env python3.10

import rospy
import cv2
import numpy as np
import math
import copy
import actionlib
import tf
import tf.transformations as tft

from sensor_msgs.msg import Image, JointState, CameraInfo
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

class GoToBagFailed(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.depth_info = None
        self.latest_rgb_msg = None
        self.latest_depth_msg = None

        # Display window for clicking
        cv2.namedWindow("Live View", cv2.WINDOW_NORMAL)

        # Sync RGB + depth
        rgb_sub = Subscriber("/xtion/rgb/image_raw", Image)
        depth_sub = Subscriber("/xtion/depth_registered/image_raw", Image)
        info_sub = Subscriber("/xtion/depth_registered/camera_info", CameraInfo)
        ats = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub], queue_size=5, slop=0.1
        )
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


    def execute(self, userdata):
        self.ask_for_bag()
        self.stow_bag()
        self.hold_gripper_position()
        return "succeeded"
        
    def ask_for_bag(self):
        self.open_gripper()
        goal = PlayMotionGoal()
        goal.motion_name = "offer_gripper"
        goal.skip_planning = False
        self.motion_client.send_goal(goal)
        self.motion_client.wait_for_result()
        rospy.loginfo("Gripper offered.")
        self.say(
            "I wasn't able to detect the bag. Please put the bag into my gripper. I will give you 10 seconds."
        )
        self.say("10")
        rospy.sleep(0.5)
        self.say("9")
        rospy.sleep(0.5)
        self.say("8")
        rospy.sleep(0.5)
        self.say("7")
        rospy.sleep(0.5)
        self.say("6")
        rospy.sleep(0.5)
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
        



if __name__ == "__main__":

    rospy.init_node("go_to_bag_failed")
    sm = smach.StateMachine(outcomes=["succeeded", "failed"])
    with sm:
        smach.StateMachine.add(
            "GO_TO_BAG",
            GoToBagFailed(),
            transitions={"succeeded": "succeeded", "failed": "failed"},
        )
    outcome = sm.execute()
    rospy.loginfo(f"Skill outcome: {outcome}")
