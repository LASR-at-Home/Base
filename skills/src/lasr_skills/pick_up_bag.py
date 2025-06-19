#!/usr/bin/env python3

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

from sensor_msgs.msg            import Image, JointState
from cv_bridge                  import CvBridge
from geometry_msgs.msg          import PoseStamped
from segment_anything           import sam_model_registry, SamPredictor
from moveit_msgs import *
from moveit_commander           import MoveGroupCommander, roscpp_initialize, roscpp_shutdown

from message_filters            import Subscriber, ApproximateTimeSynchronizer
from control_msgs.msg           import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg        import JointTrajectory, JointTrajectoryPoint
from pal_interaction_msgs.msg   import TtsAction, TtsGoal, TtsText
from play_motion_msgs.msg       import PlayMotionAction, PlayMotionGoal
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
import std_srvs.srv


class BagPickAndPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        # --- SAM2 setup ---
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        root = os.path.dirname(__file__)
        pkg  = os.path.dirname(root)
        ckpt = os.path.join(pkg, "models", "sam_vit_b_01ec64.pth")
        sam  = sam_model_registry["vit_b"](checkpoint=ckpt)
        sam.to(self.device)
        self.predictor = SamPredictor(sam)
        # -------------------

        self.bridge       = CvBridge()
        self.latest_rgb   = None
        self.latest_depth = None
        self.click        = None

        # Display window for clicking
        cv2.namedWindow("Live View", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Live View", self.on_mouse_click)

        # Sync RGB + depth
        rgb_sub   = Subscriber('/xtion/rgb/image_raw', Image)
        depth_sub = Subscriber('/xtion/depth/image_raw', Image)
        ats = ApproximateTimeSynchronizer([rgb_sub, depth_sub],
                                          queue_size=5, slop=0.1)
        ats.registerCallback(self.synced_callback)

        # --- MoveIt! arm setup ---
        roscpp_initialize([])
        self.arm = MoveGroupCommander('arm_torso')

        # --- Gripper action client ---
        self.gripper_client = actionlib.SimpleActionClient(
            '/parallel_gripper_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        rospy.loginfo("Waiting for gripper action server…")
        self.gripper_client.wait_for_server()
        rospy.loginfo("Gripper action server ready.")

        self.motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
        rospy.loginfo("Waiting for /play_motion action server...")
        self.motion_client.wait_for_server()
        rospy.loginfo("/play_motion action server connected")

        self.torso_client = actionlib.SimpleActionClient(
            '/torso_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        rospy.loginfo("Waiting for torso action server…")
        self.torso_client.wait_for_server()
        rospy.loginfo("Torso action server ready.")

        self.tts_client = actionlib.SimpleActionClient('tts', TtsAction)
        rospy.loginfo("Waiting for TTS action server…")
        self.tts_client.wait_for_server()
        rospy.loginfo("TTS action server connected.")

    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click = (x, y)
            rospy.loginfo(f"User click at {self.click}")

    def synced_callback(self, rgb_msg, depth_msg):
        try:
            self.latest_rgb   = self.bridge.imgmsg_to_cv2(rgb_msg,   'bgr8')
            self.latest_depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
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
                    return 'failed'

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
                    return 'failed'

            if not self.click:
                rospy.logwarn("No click received.")
                cv2.destroyWindow("Live View")
                return 'failed'

            u, v = self.click
            self.click = None

            # 1) segment with SAM
            self.predictor.set_image(frame)
            coords = np.array([[u, v]], dtype=float)
            labs   = np.array([1], dtype=int)
            masks, _, _ = self.predictor.predict(
                point_coords=coords,
                point_labels=labs,
                multimask_output=False
            )
            mask = masks[0].astype(bool)

            pts = []
            h, w = mask.shape
            fx = fy = 579.653076171875   # verify with camera_info!
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
                return 'failed'
            pts = np.array(pts)
            centroid = np.median(pts, axis=0)
            rospy.loginfo(f"Bag centroid (m): {centroid}")
            xy = pts[:, :2] - centroid[:2]
            cov = np.cov(xy, rowvar=False)
            evals, evecs = np.linalg.eigh(cov)
            long_axis = evecs[:, np.argmax(evals)]
            orth = np.array([-long_axis[1], long_axis[0]])
            orth /= np.linalg.norm(orth)
            yaw  = math.atan2(orth[1], orth[0])

            self.prepare_pick()
            result = self.pick(centroid, yaw)
            if not result:
                self.ask_for_bag()

            self.stow_bag()
            cv2.destroyWindow("Live View")
            return 'succeeded'
        except Exception as e:
            rospy.logerr(f"Exception in skill: {e}")
            try:
                cv2.destroyWindow("Live View")
            except:
                pass
            return 'failed'

    def adjust_torso(self,target_height):
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
        goal.motion_name = 'prepare_grasp'
        goal.skip_planning = False
        self.motion_client.send_goal(goal)
        self.motion_client.wait_for_result()
        self.adjust_torso(0.1)
        rospy.loginfo("Reached prepare grasp pose.")

    def pick(self, centroid_mm, yaw):
        x_mm, y_mm, z_mm = centroid_mm
        x_cam, y_cam, z_cam = x_mm/1000.0, y_mm/1000.0, z_mm/1000.0

        listener = tf.TransformListener()
        listener.waitForTransform(
            'base_footprint',
            'xtion_rgb_optical_frame',
            rospy.Time(0),
            rospy.Duration(1.0)
        )
        trans, rot = listener.lookupTransform(
            'base_footprint',
            'xtion_rgb_optical_frame',
            rospy.Time(0)
        )
        T = tft.quaternion_matrix(rot)
        T[0:3, 3] = trans
        x, y, z = T.dot([x_cam, y_cam, z_cam, 1.0])[:3]

        close_cam = np.array([ math.cos(yaw), math.sin(yaw), 0.0 ])
        R_cam2base = tft.quaternion_matrix(rot)[0:3, 0:3]
        close_base = R_cam2base.dot(close_cam)
        close_base[2] = 0.0
        close_base   /= np.linalg.norm(close_base)
        yaw_base = math.atan2(close_base[1], close_base[0])
        q_final = tft.quaternion_from_euler(0,  math.pi/2, yaw_base)

        pose = PoseStamped()
        pose.header.frame_id = 'base_footprint'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z*2 # Top point is twice the height of the centroid

        pose.pose.orientation.x = float(q_final[0])
        pose.pose.orientation.y = float(q_final[1])
        pose.pose.orientation.z = float(q_final[2])
        pose.pose.orientation.w = float(q_final[3])

        if pose.pose.position.z < 0.6:
            pose.pose.position.z = 0.6

        rospy.loginfo(f"Final pick pose in base_footprint: {pose}")

        self.arm.set_start_state_to_current_state()
        pre_pose = copy.deepcopy(pose)
        pre_pose.pose.position.z += 0.20
        self.arm.set_pose_target(pre_pose)
        self.arm.go(wait=True)
        self.arm.clear_pose_targets()
        rospy.sleep(0.2)

        self.run_clear_octomap()
        result = self.sync_shift_ee(self.arm, 0.35, 0.0, 0.0)
        if result == False:
            return result
        self.close_gripper()
        rospy.loginfo("Bag pick complete!")
        self.sync_shift_ee(self.arm, -0.35, 0.0, 0.0)

        return self.is_picked_up(0.001, 0.25)

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

    def run_clear_octomap(self):
        rospy.wait_for_service('/clear_octomap', timeout=2.0)
        clear_octomap = rospy.ServiceProxy('/clear_octomap', std_srvs.srv.Empty)
        clear_octomap()
        rospy.sleep(0.1)
        rospy.loginfo("Octomap cleared before executing move.")

    def is_picked_up(self, pos_thresh=0.001, effort_thresh=0.05):
        rospy.sleep(0.1)
        js = rospy.wait_for_message('/joint_states', JointState, timeout=1.0)
        lidx = js.name.index('gripper_left_finger_joint')
        ridx = js.name.index('gripper_right_finger_joint')
        pos_l, pos_r   = js.position[lidx], js.position[ridx]
        eff_l, eff_r   = js.effort[lidx],   js.effort[ridx]
        avg_pos   = 0.5 * (pos_l + pos_r)
        avg_eff   = abs(0.5 * (eff_l + eff_r))
        if avg_pos > pos_thresh and avg_eff > effort_thresh:
            rospy.loginfo(f"Grasp succeeded: pos={avg_pos:.3f} m, eff={avg_eff:.2f}")
            return True
        else:
            rospy.logwarn(f"No object: pos={avg_pos:.3f} m, eff={avg_eff:.2f}")
            return False

    def ask_for_bag(self):
        self.open_gripper()
        goal = PlayMotionGoal()
        goal.motion_name = 'offer_gripper'
        goal.skip_planning = False
        self.motion_client.send_goal(goal)
        self.motion_client.wait_for_result()
        rospy.loginfo("Gripper offered.") 
        self.say("I wasn't able to pickup the bag. Please put the bag into my gripper. I will give you 5 seconds.")
        rospy.sleep(5)
        self.close_gripper()

    def say(self, text: str):
        if hasattr(self, 'tts_client'):
            goal = TtsGoal(rawtext=TtsText(text=text, lang_id="en_GB"))
            self.tts_client.send_goal(goal)
            self.tts_client.wait_for_result()
        else:
            rospy.loginfo(f"[TTS fallback] {text}")

    def stow_bag(self):
        self.adjust_torso(0.3)
        goal = PlayMotionGoal()
        goal.motion_name = 'stow_bag'
        goal.skip_planning = False
        self.motion_client.send_goal(goal)
        self.motion_client.wait_for_result()
        rospy.loginfo("Reached stow pose.") 

    def close_gripper(self, width=0.00, duration=1.0):
        goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = ['gripper_joint']
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
        traj.joint_names = ['gripper_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [width]
        pt.time_from_start = rospy.Duration(duration)
        traj.points = [pt]
        goal.trajectory = traj
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()


if __name__ == '__main__':
    import smach
    rospy.init_node('bag_pick_and_place_skill_runner')
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    with sm:
        smach.StateMachine.add('BAG_PICK_AND_PLACE', BagPickAndPlace(),
                               transitions={'succeeded': 'succeeded',
                                            'failed': 'failed'})
    outcome = sm.execute()
    rospy.loginfo(f"Skill outcome: {outcome}")
    roscpp_shutdown()
