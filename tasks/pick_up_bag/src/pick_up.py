#!/usr/bin/env python3.10
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

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

from sensor_msgs.msg            import Image, JointState
from cv_bridge                  import CvBridge
from geometry_msgs.msg          import PoseStamped
from segment_anything           import sam_model_registry, SamPredictor
from moveit_commander           import MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from message_filters            import Subscriber, ApproximateTimeSynchronizer
from control_msgs.msg           import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg        import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from pal_interaction_msgs.msg import TtsAction, TtsGoal, TtsText
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest


class BagPickAndPlaceNode:
    def __init__(self):
        rospy.init_node('bag_pick_and_place')

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
        # TIAGo’s torso+arm planning group is named "arm_torso" :contentReference[oaicite:2]{index=2}
        self.arm = MoveGroupCommander('arm_torso')

        # --- Gripper action client ---
        # TIAGo steel version exposes /parallel_gripper_controller/follow_joint_trajectory :contentReference[oaicite:3]{index=3}
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

        rospy.loginfo("BagPickAndPlaceNode ready.")
        self.loop()

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

    def loop(self):
        self.adjust_torso(0.1)

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.latest_rgb is None:
                rate.sleep()
                continue

            frame = self.latest_rgb.copy()
            cv2.imshow("Live View", frame)

            if self.click:
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

                # filter noise

                # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
                # mask_clean = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, kernel)
                # mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)
                # mask = mask_clean.astype(bool)

                # depth_smoothed = cv2.medianBlur(self.latest_depth, 5)



                # 2) project masked pixels into 3D (meters!)
                pts = []
                h, w = mask.shape
                fx = fy = 579.653076171875   # verify with camera_info!
                cx, cy = 319.5, 239.5
                for yy, xx in zip(*np.where(mask)):
                    z = self.latest_depth[yy, xx]    # in meters
                    # z = depth_smoothed[yy, xx]
                    if np.isfinite(z) and z > 0.05:  # ignore closer than 5cm
                        X = (xx - cx) * z / fx        # in meters
                        Y = (yy - cy) * z / fy
                        pts.append((X, Y, z))

                if not pts:
                    rospy.logwarn("No valid 3D points—try clicking elsewhere.")
                else:
                    pts = np.array(pts)

                    # 3) centroid = median for robustness
                    centroid = np.median(pts, axis=0)
                    rospy.loginfo(f"Bag centroid (m): {centroid}")

                    # 4) PCA → long axis in XY
                    xy = pts[:, :2] - centroid[:2]
                    cov = np.cov(xy, rowvar=False)
                    evals, evecs = np.linalg.eigh(cov)
                    long_axis = evecs[:, np.argmax(evals)]

                    # 5) yaw that spins the wrist across the bag’s span
                    orth = np.array([-long_axis[1], long_axis[0]])
                    orth /= np.linalg.norm(orth)
                    yaw  = math.atan2(orth[1], orth[0])

                    # 6) prep + pick
                    self.prepare_pick()
                    result = self.pick(centroid, yaw)

                    if not result:
                        self.ask_for_bag()

                    # stow the bag
                    self.stow_bag()
  
                # 5a) overlay mask so you know what you clicked
                colored = np.stack([mask*255]*3, axis=-1).astype(np.uint8)
                overlay = cv2.addWeighted(frame, 0.7, colored, 0.3, 0)
                cv2.imshow("Live View", overlay)

            cv2.waitKey(1)
            rate.sleep()

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
        goal.motion_name = 'prepare_grasp'  # must be defined in your play_motion config
        goal.skip_planning = False
        self.motion_client.send_goal(goal)
        self.motion_client.wait_for_result()
        self.adjust_torso(0.1)
        rospy.loginfo("Reached prepare grasp pose.")

        # self.adjust_torso(0.15)
        # self.arm.set_start_state_to_current_state()
        # self.arm.set_planner_id("RRTConnectkConfigDefault")

        # setup_pose = PoseStamped()
        # setup_pose.header.frame_id = "base_footprint"
        # setup_pose.header.stamp = rospy.Time.now()

        # setup_pose.pose.position.x = 0.508
        # setup_pose.pose.position.y = -0.408
        # setup_pose.pose.position.z = 0.932

        # setup_pose.pose.orientation.x = 0.005
        # setup_pose.pose.orientation.y = -0.052
        # setup_pose.pose.orientation.z = 0.039
        # setup_pose.pose.orientation.w = 0.9999999969548019

        # down vector quaternion
        # setup_pose.pose.orientation.x = 0.0010062689510449319
        # setup_pose.pose.orientation.y = 0.70710678
        # setup_pose.pose.orientation.z = 0.0004933099788376551
        # setup_pose.pose.orientation.w = 0.70710678

        # self.arm.set_pose_target(setup_pose)
        # success, traj, _, err = self.arm.plan()
        # if not success:
        #     rospy.logerr(f"prepare_pick plan failed ({err})")
        # self.arm.execute(traj, wait=True)
        # self.arm.stop()
        # self.arm.clear_pose_targets()
        # rospy.loginfo("Reached setup pose.")

        # self.adjust_torso(0.1)



    def pick(self, centroid_mm, yaw):
        # 1) centroid mm→m
        x_mm, y_mm, z_mm = centroid_mm
        x_cam, y_cam, z_cam = x_mm/1000.0, y_mm/1000.0, z_mm/1000.0

        # 2) lookup camera→base translation only
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

        # 3) build orientation in base frame: roll=0, pitch=-90°, yaw=your PCA
        close_cam = np.array([ math.cos(yaw), math.sin(yaw), 0.0 ])   # XY only

        # b. Rotate that vector into base frame with the same rot we used above
        #    (rot is the quaternion camera→base)
        R_cam2base = tft.quaternion_matrix(rot)[0:3, 0:3]             # 3×3
        close_base = R_cam2base.dot(close_cam)
        # project back onto base XY plane and normalise
        close_base[2] = 0.0
        close_base   /= np.linalg.norm(close_base)

        # c. Base-frame yaw = atan2(Y, X) of that transformed vector
        yaw_base = math.atan2(close_base[1], close_base[0])

        # d. Build final quaternion: roll = 0, pitch = +90° (down), yaw = yaw_base
        q_final = tft.quaternion_from_euler(0,  math.pi/2, yaw_base)


        
        # 4) assemble final pick pose
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

        print(pose)

        if pose.pose.position.z < 0.6:
            pose.pose.position.z = 0.6

        rospy.loginfo(f"Final pick pose in base_footprint: {pose}")

        # 5) Execute the usual pre-grasp → grasp sequence
        self.arm.set_start_state_to_current_state()

        # pre-grasp: 10 cm above
        self.arm.clear_pose_targets()
        pre_pose = copy.deepcopy(pose)
        pre_pose.pose.position.z += 0.20
        self.arm.set_pose_target(pre_pose)
        self.arm.go(wait=True)
        self.arm.clear_pose_targets()
        rospy.sleep(0.2)

        # move down by 0.35
        self.sync_shift_ee(self.arm, 0.35, 0.0, 0.0)
        # grasp
        self.close_gripper()
        rospy.loginfo("Bag pick complete!")

        # move up by 0.35
        self.sync_shift_ee(self.arm, -0.35, 0.0, 0.0)

        return self.is_picked_up()

    def sync_shift_ee(self, move_group, x, y, z):
        from tf.transformations import euler_from_quaternion, euler_matrix

        curr_pose = move_group.get_current_pose()
        pose = curr_pose.pose
        # print('x: {}, y: {}, z: {}'.format(pose.position.x, pose.position.y, pose.position.z))

        quat = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        euler = euler_from_quaternion(quat)
        euler_m = euler_matrix(*euler)

        # calculate offset to add to the current pose
        delta = np.dot(euler_m[:3, :3], np.array([x, y, z]).T)
        pose.position.x += delta[0]
        pose.position.y += delta[1]
        pose.position.z += delta[2]

        move_group.set_pose_target(curr_pose)
        # publish pose for debugging purposes
        # print('Publishing debug_plan_pose')

        move_group.set_start_state_to_current_state()
        result = move_group.go(wait=True)
        return result

    def is_picked_up(self, pos_thresh=0.001, effort_thresh=0.05):
        js = rospy.wait_for_message('/joint_states', JointState, timeout=1.0)
        # find indices
        lidx = js.name.index('gripper_left_finger_joint')
        ridx = js.name.index('gripper_right_finger_joint')

        # 3) compute average position & effort
        pos_l, pos_r   = js.position[lidx], js.position[ridx]
        eff_l, eff_r   = js.effort[lidx],   js.effort[ridx]
        avg_pos   = 0.5 * (pos_l + pos_r)
        avg_eff   = abs(0.5 * (eff_l + eff_r))

        # 4) decide if we’re blocked
        if avg_pos > pos_thresh and avg_eff > effort_thresh:
            rospy.loginfo(f"Grasp succeeded: pos={avg_pos:.3f} m, eff={avg_eff:.2f}")
            return True
        else:
            rospy.logwarn(f"No object: pos={avg_pos:.3f} m, eff={avg_eff:.2f}")
            return False


    # def close_gripper(self):
    #     goal = PlayMotionGoal()
    #     goal.motion_name = 'close_gripper'  # must be defined in your play_motion config
    #     goal.skip_planning = True
    #     self.motion_client.send_goal(goal)
    #     self.motion_client.wait_for_result()

    # def open_gripper(self):
    #     goal = PlayMotionGoal()
    #     goal.motion_name = 'open_gripper'  # must be defined in your play_motion config
    #     goal.skip_planning = True
    #     self.motion_client.send_goal(goal)
    #     self.motion_client.wait_for_result()     

    def ask_for_bag(self):
        self.open_gripper()
        goal = PlayMotionGoal()
        goal.motion_name = 'offer_gripper'  # must be defined in your play_motion config
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
        # goal.motion_name = 'hold_object_home'  # must be defined in your play_motion config
        goal.motion_name = 'stow_bag'  # must be defined in your play_motion config
        goal.skip_planning = False
        self.motion_client.send_goal(goal)
        self.motion_client.wait_for_result()
        rospy.loginfo("Reached stow pose.") 


    def close_gripper(self, width=0.00, duration=1.0):
        # width = final opening (m) between fingers; 0.0 = fully closed
        goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = ['gripper_joint']  # adjust if your joint is named differently
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
    try:
        BagPickAndPlaceNode()
    finally:
        roscpp_shutdown()
