#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import numpy as np
import ros_numpy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image as SensorImage, PointCloud2, CameraInfo
from lasr_vision_msgs.srv import LangSam, LangSamRequest
import sensor_msgs.point_cloud2 as pc2

import rospy
import smach
import smach_ros

from lasr_vision_msgs.srv import LangSam, LangSamRequest
from sensor_msgs.msg import Image

import numpy 

#

import rospy
import smach
from lasr_vision_msgs.srv import LangSam, LangSamRequest
from sensor_msgs.msg import Image, CameraInfo

from geometry_msgs.msg import Point

class LangSamState(smach.State):
    def __init__(self, prompt="door handle", target_frame="base_footprint"):  # fixed spelling
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=['prompt', 'target_point'],
            output_keys=['detections', 'target_point']
        )
        self.lang_sam_srv = rospy.ServiceProxy("/lasr_vision/lang_sam", LangSam)
        self.target_frame = target_frame
        self.prompt = prompt
        self.initialized = False

    def initialize(self):
        rospy.wait_for_service("/lasr_vision/lang_sam")
        self.initialized = True

    def execute(self, userdata):
        if not self.initialized:
            self.initialize()

        try:
            image_msg = rospy.wait_for_message('/xtion/rgb/image_raw', Image, timeout=5.0)
            depth_msg = rospy.wait_for_message('/xtion/depth_registered/image_raw', Image, timeout=5.0)
            cam_info = rospy.wait_for_message('/xtion/depth_registered/camera_info', CameraInfo, timeout=5.0)

            request = LangSamRequest()
            request.image_raw = image_msg
            request.depth_image = depth_msg
            request.depth_camera_info = cam_info
            request.prompt = self.prompt
            request.target_frame = self.target_frame
            request.box_threshold = 0.3
            request.text_threshold = 0.25

            response = self.lang_sam_srv(request)

            if not response.detections:
                rospy.logwarn("LangSAM: No detections found.")
                return 'failed'
            
            userdata.detections = response.detections
            best = max(response.detections, key=lambda d: d.seg_mask_score)
            userdata.target_point = best.point

            rospy.loginfo(f"{userdata.target_point}")



            return 'succeeded'

        except rospy.ServiceException as e:
            rospy.logerr(f"LangSAM service call failed: {e}")
            return 'failed'
        except rospy.ROSException as e:
            rospy.logerr(f"Timeout waiting for image/depth/camera info: {e}")
            return 'failed'

import smach
import rospy
import actionlib
import moveit_commander
from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction
from geometry_msgs.msg import Point

import sys
import copy
import rospy
import actionlib
import moveit_commander
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker
from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from moveit_commander.exception import MoveItCommanderException
from trajectory_msgs.msg import JointTrajectory
import math

USE_CLICKED_POINT = True      
TORSO_LIFT        = 0.25       
TABLE_THICK       = 0.01      
TABLE_Z           = 0.41       
VEL_SCALE         = 0.15     
ACC_SCALE         = 0.15
STAMP_DELAY       = 0.25 

class GoToOpenPoseState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed"],
                             input_keys=["target_point"])

        self.gripper_link = "gripper_tool_link"
        self.arm = moveit_commander.MoveGroupCommander("arm_torso")
        self.arm.set_end_effector_link(self.gripper_link)
        self.arm.set_pose_reference_frame("base_footprint")
        self.arm.set_planning_time(30.0)
        self.arm.set_max_velocity_scaling_factor(0.15)
        self.arm.set_max_acceleration_scaling_factor(0.15)
        self.arm.set_goal_tolerance(0.01)

        self.play = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
        self.play.wait_for_server()

    def execute(self, userdata):
        try:
            p: Point = userdata.target_point
            self.arm.set_position_target([p.x, p.y, p.z], self.gripper_link)

            ok, plan, _, _ = self.arm.plan()
            if not ok or not self.arm.execute(plan, wait=True):
                rospy.logerr("Move to target failed")
                return "failed"

            self.arm.stop()
            self.arm.clear_pose_targets()

            # Open gripper
            open_goal = PlayMotionGoal()
            open_goal.motion_name = "open_gripper"
            open_goal.skip_planning = False
            self.play.send_goal_and_wait(open_goal)

            return "succeeded"
        

        except Exception as e:
            rospy.logerr(f"GoToOpenPose failed: {e}")
            return "failed"


# class LangSamState(smach.State):
#     def __init__(self, prompt: str, image_topic: str = "/xtion/rgb/image_raw"):
#         smach.State.__init__(self, outcomes=["succeeded", "failed"], output_keys=['sam_detections'])

#         self.prompt = prompt
#         self.image_topic = image_topic
#         self.lang_sam_topic = "/lasr_vision/lang_sam"
#         self.depth_image_topic = "/xtion/depth_registered/image_raw"
#         self.depth_camera_topic = "/xtion/depth_registered/camera_info"
#         self.target_frame = "map"

#         try:
#             rospy.loginfo("[LangSamState] Waiting for LangSAM service")
#             rospy.wait_for_service(self.lang_sam_topic, timeout=5)
#             self.client = rospy.ServiceProxy(self.lang_sam_topic, LangSam)
#             rospy.loginfo("[LangSamState] Loaded LangSAM service")
#         except rospy.ROSException:
#             rospy.logerr(f"[LangSamState] Timeout: LangSAM service")

#     def execute(self, userdata):

#         try:
#             rospy.loginfo(f"[LangSamState] Waiting for {self.image_topic}")
#             image_msg = rospy.wait_for_message(self.image_topic, Image, timeout=5)
#             rospy.loginfo(f"[LangSamState] Loaded {self.image_topic}")
#             depth_image_msg = rospy.wait_for_message(self.depth_image_topic, Image, timeout=5)
#             rospy.loginfo(f"[LangSamState] Loaded {self.depth_image_topic}")
#             camera_info_msg = rospy.wait_for_message(self.depth_camera_topic, CameraInfo, timeout=5)
#             rospy.loginfo(f"[LangSamState] Loaded {self.depth_camera_topic}")


#         except rospy.ROSException:
#             rospy.logerr(f"[LangSamState] Timeout: {self.image_topic}")
#             return "failed"
            
#         request = LangSamRequest(
#                 image_raw=image_msg,
#                 depth_image=depth_image_msg,
#                 depth_camera_info=camera_info_msg,
#                 target_frame=self.target_frame,
#                 prompt=self.prompt,
#             )

#         try:
#             response = self.client(request)


#             if not response.detections:           
#                 rospy.logwarn("[LangSamState] No detection from LangSam")
#                 return "failed" 
            
#             rospy.loginfo(f"[LangSamState] LangSAM result: {len(response.detections)}")

#             best_idx = numpy.argmax([d.seg_mask_score for d in response.detections])
#             detection = response.detections[best_idx]
#             point = detection.point

#             rospy.logwarn(f"[LangSamState] {point}")

#             # width = image_msg.width
#             # height = image_msg.height

#             # flat_mask = numpy.array(detection.seg_mask, dtype=numpy.uint8)
#             # mask_np = flat_mask.reshape((height, width))


#             # # if numpy.count_nonzero(mask_np) < 5000:
#             # #     rospy.logwarn("[LangSamState] Mask too small")
#             # #     return "failed"

#             # # userdata.sam_detections = {
#             # #     "mask": mask_np,
#             # #     "image_header": image_msg.header
#             # # }

#             userdata.sam_detections = {
#                 "point": point
#             }

#             return "succeeded"

#         except rospy.ServiceException as e:
#             rospy.logerr(f"[LangSamState] LangSAM service call failed: {e}")
#             return "failed"

#

# def filter_point_cloud(mask, cloud_msg):
#         height = cloud_msg.height
#         width = cloud_msg.width

#         if mask.shape != (height, width):
#             rospy.logwarn("[filter_point_cloud] Mask shape mismatch.")
#             return []
            
#         cloud_iter = pc2.read_points(cloud_msg, skip_nans=False, field_names=("x", "y", "z"))
#         cloud_array = np.array(list(cloud_iter), dtype=np.float32).reshape((height, width, 3))
        
#         selected_points = cloud_array[mask.astype(bool)]
#         selected_points = selected_points[~np.isnan(selected_points).any(axis=1)]

#         rospy.loginfo(f"[filter_point_cloud] Masked pixels: {np.count_nonzero(mask)}, Valid 3D points: {len(selected_points)}")

#         return selected_points.tolist()

# class FilterPointCloudState(smach.State):
#     def __init__(self, cloud_topic="/xtion/depth_pointsros"):
#         smach.State.__init__(self, outcomes=["succeeded", "failed"], input_keys=["sam_detections"], output_keys=["sam_detections"])
#         self.cloud_topic = cloud_topic

#     def execute(self, userdata):
#         try:
#             rospy.loginfo(f"[FilterPointCloudState] Waiting for {self.cloud_topic}")
#             cloud_msg = rospy.wait_for_message(self.cloud_topic, PointCloud2, timeout=5)
#             rospy.loginfo(f"[FilterPointCloudState] Received {self.cloud_topic}")
#         except rospy.ROSException:
#             rospy.logerr(f"[FilterPointCloudState] Timeout: {self.cloud_topic}")
#             return "failed"
        
#         mask = userdata.sam_detections["mask"]
#         filtered_points = self.filter_point_cloud(mask, cloud_msg)

#         rospy.loginfo(f"[FilterPointCloudState] Filtered {len(filtered_points)} points")
#         userdata.sam_detections["filtered_points"] = filtered_points
#         userdata.sam_detections["raw_points"]=cloud_msg
#         return "succeeded"
    
    

# # #

# # class PrintPointCountState(smach.State):
# #     def __init__(self):
# #         smach.State.__init__(self, outcomes=["done", "failed"], input_keys=["sam_detections"])

# #     def execute(self, userdata):
# #         points = userdata.sam_detections.get("filtered_points", [])
# #         rospy.loginfo(f"[PrintPointCountState] 3D points count: {len(points)}")
# #         return "done"
    
# # #

# def select_depth_with_ratio(points, percentage = 75):
#     depths = numpy.array(points)
#     depths = depths[depths > 0]
#     if len(depths) == 0:
#         return None
#     return np.percentile(depths, percentage)

# class EstimateAverageDepthState(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=["done"], input_keys=["sam_detections"], output_keys=["sam_detections"])

#     def execute(self, userdata):
#         points = userdata.sam_detections.get("points_3d", [])
#         if not points:
#             rospy.logwarn("[EstimateAverageDepthState] No 3D points to process.")
#             userdata.sam_detections["average_depth"] = None
#             return "done"

#         z_values = [p[2] for p in points if not np.isnan(p[2])]
#         if not z_values:
#             rospy.logwarn("[EstimateAverageDepthState] All depth values are NaN.")
#             userdata.sam_detections["average_depth"] = None
#             return "done"

#         door_depth = select_depth_with_ratio(z_values, 75)
#         handle_depth = np.min(z_values)

#         rospy.loginfo(f"[EstimateAverageDepthState] Avg depth: {avg_depth:.3f} m, Min: {min_depth:.3f} m, Max: {max_depth:.3f} m")

#         userdata.sam_detections["average_depth"] = avg_depth
#         userdata.sam_detections["min_depth"] = min_depth
#         userdata.sam_detections["max_depth"] = max_depth
#         return "done"

# import rospy
# import smach
# import numpy as np
# from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# import tf
# from sklearn.decomposition import PCA

# class DetectHandleState(smach.State):
#     def __init__(self, reference_frame="base_footprint"):
#         smach.State.__init__(self,
#                              outcomes=["succeeded", "failed"],
#                              input_keys=["sam_detections"],
#                              output_keys=["handle_pose_stamped", "sam_detections"])
#         self.reference_frame = reference_frame

#     def execute(self, userdata):
#         rospy.loginfo("[DetectHandleState] Starting grasp analysis...")

#         try:
#             points_3d = np.array(userdata.sam_detections["points_3d"])
#         except KeyError:
#             rospy.logerr("[DetectHandleState] No 3D points found in userdata.")
#             return "failed"

#         if points_3d.shape[0] == 0:
#             rospy.logwarn("[DetectHandleState] Empty 3D point cloud.")
#             return "failed"

#         # ✅ Step 1: Compute 3D center of mass
#         center = np.mean(points_3d, axis=0)
#         rospy.loginfo(f"[DetectHandleState] 3D center: {center}")

#         # ✅ Step 2: Tight spatial filtering (±5 cm box)
#         spatial_margin = 0.05
#         spatial_filtered = points_3d[np.all(np.abs(points_3d - center) < spatial_margin, axis=1)]
#         rospy.loginfo(f"[DetectHandleState] Spatially filtered points: {spatial_filtered.shape[0]}")

#         # ✅ Step 3: Optional depth filtering (±1 cm)
#         depth_center = center[2]
#         depth_margin = 0.01
#         depth_filtered = spatial_filtered[np.abs(spatial_filtered[:, 2] - depth_center) < depth_margin]
#         rospy.loginfo(f"[DetectHandleState] After depth filtering: {depth_filtered.shape[0]}")

#         if depth_filtered.shape[0] < 10:
#             rospy.logwarn("[DetectHandleState] Not enough points after filtering.")
#             return "failed"

#         # ✅ Step 4: PCA on filtered points
#         centered = depth_filtered - np.mean(depth_filtered, axis=0)
#         pca = PCA(n_components=3)
#         pca.fit(centered)
#         transformed = pca.transform(centered)

#         width = np.max(transformed[:, 0]) - np.min(transformed[:, 0])
#         height = np.max(transformed[:, 1]) - np.min(transformed[:, 1])

#         # ✅ Step 5: Sanity check
#         if width > 0.3:
#             rospy.logwarn(f"[DetectHandleState] Warning: large handle width: {width:.3f} m — possibly over-segmented.")

#         rospy.loginfo(f"[DetectHandleState] Estimated handle width: {width:.4f} m")
#         rospy.loginfo(f"[DetectHandleState] Estimated handle height: {height:.4f} m")

#         # ✅ Step 6: Create PoseStamped
#         quat = tf.transformations.quaternion_from_euler(0, np.pi, 0)
#         pose_stamped = PoseStamped()
#         pose_stamped.header.stamp = rospy.Time.now()
#         pose_stamped.header.frame_id = self.reference_frame
#         pose_stamped.pose = Pose(
#             position=Point(*center.tolist()),
#             orientation=Quaternion(*quat)
#         )

#         # ✅ Step 7: Store in userdata
#         userdata.handle_pose_stamped = pose_stamped
#         userdata.sam_detections["handle_width"] = float(width)
#         userdata.sam_detections["handle_height"] = float(height)

#         return "succeeded"


    
# import rospy
# import smach
# import numpy as np
# from sklearn.decomposition import PCA

# class DetectHandleState(smach.State):
#     def __init__(self):
#         smach.State.__init__(
#             self,
#             outcomes=["done", "failed"],
#             input_keys=["sam_detections"],
#             output_keys=["handle_pose"]
#         )

#     def execute(self, userdata):
#         points = userdata.sam_detections.get("points_3d", [])

#         if not points:
#             rospy.logwarn("[DetectHandleState] No 3D points provided.")
#             return "failed"

#         points_np = np.array(points)

#         try:
#             pca_all = PCA(n_components=3)
#             pca_all.fit(points_np)
#             proj_all = pca_all.transform(points_np)

#             # Step 2: 첫 번째 주성분 방향 기준으로 중심 근처 슬라이싱 (2cm 이내)
#             center_proj_0 = proj_all[:, 0]
#             handle_mask = np.abs(center_proj_0) < 0.02  # ±2cm 슬라이싱
#             handle_points = points_np[handle_mask]

#             if len(handle_points) < 10:
#                 rospy.logwarn("[DetectHandleState] Not enough points after PCA-based slicing.")
#                 return "failed"

#             # Step 3: 슬라이싱된 점들로 2차 PCA → 정확한 크기 측정
#             pca = PCA(n_components=3)
#             pca.fit(handle_points)
#             center = pca.mean_

#             proj = pca.transform(handle_points)

#             # Step 4: 너비와 높이 계산 (축 1, 2 기준)
#             width = proj[:, 1].max() - proj[:, 1].min()
#             height = proj[:, 2].max() - proj[:, 2].min()
#             handle_width = min(width, height)

#             # 결과 출력 및 저장
#             rospy.loginfo(f"[DetectHandleState] Handle width (axis 1): {width:.3f} m")
#             rospy.loginfo(f"[DetectHandleState] Handle height (axis 2): {height:.3f} m")
#             rospy.loginfo(f"[DetectHandleState] Selected handle width (min): {handle_width:.3f} m")

#             userdata.handle_pose = {
#                 "position": center.tolist(),
#                 "width": float(handle_width)
#             }
#             return "done"

#         except Exception as e:
#             rospy.logerr(f"[DetectHandleState] Error during handle detection: {e}")
#             return "failed"
       
        
# import rospy
# import smach
# import actionlib
# from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
# from trajectory_msgs.msg import JointTrajectoryPoint

# class RaiseTorsoState(smach.State):
#     def __init__(self, height=0.20, duration=3.0):
#         """
#         Raises TIAGo's torso to the specified height (in meters).
#         Default is 0.15 m, which provides decent reach.
#         """
#         smach.State.__init__(self, outcomes=["succeeded", "failed"])
#         self.height = height
#         self.duration = duration

#         self.client = actionlib.SimpleActionClient(
#             '/torso_controller/follow_joint_trajectory',
#             FollowJointTrajectoryAction
#         )

#         rospy.loginfo("[RaiseTorsoState] Waiting for torso_controller action server...")
#         self.client.wait_for_server()
#         rospy.loginfo("[RaiseTorsoState] Connected to torso_controller.")

#     def execute(self, userdata):
#         rospy.loginfo(f"[RaiseTorsoState] Raising torso to {self.height:.2f} m...")

#         goal = FollowJointTrajectoryGoal()
#         goal.trajectory.joint_names = ['torso_lift_joint']

#         point = JointTrajectoryPoint()
#         point.positions = [self.height]
#         point.time_from_start = rospy.Duration(self.duration)
#         goal.trajectory.points = [point]

#         goal.trajectory.header.stamp = rospy.Time.now()

#         self.client.send_goal(goal)
#         success = self.client.wait_for_result(rospy.Duration(self.duration + 2.0))

#         if success and self.client.get_result():
#             rospy.loginfo("[RaiseTorsoState] Torso motion succeeded.")
#             return "succeeded"
#         else:
#             rospy.logwarn("[RaiseTorsoState] Failed to move torso.")
#             return "failed"

# import rospy
# import smach
# import numpy as np
# from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# import moveit_commander
# from tf.transformations import quaternion_from_euler

# import rospy
# import smach
# import numpy as np
# import moveit_commander

# from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# from tf.transformations import quaternion_from_euler
# import tf2_ros
# import tf2_geometry_msgs


# class MoveToHandleState(smach.State):
#     def __init__(self, eef_offset: float = 0.15):
#         smach.State.__init__(
#             self,
#             outcomes=["succeeded", "failed"],
#             input_keys=["handle_pose"]
#         )

#         self._eef_offset = eef_offset

#         self._group = moveit_commander.MoveGroupCommander("arm_torso")
#         self._group.set_planner_id("RRTConnectkConfigDefault")
#         self._group.set_pose_reference_frame("base_footprint")
#         self._group.allow_replanning(True)
#         self._group.set_planning_time(30)
#         self._group.set_num_planning_attempts(40)

#         self._debug_pub = rospy.Publisher("/debug_target_pose", PoseStamped, queue_size=1)

#         # TF2 setup
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

#     def execute(self, userdata):
#         handle_pose = userdata.handle_pose
#         if not handle_pose or "position" not in handle_pose:
#             rospy.logerr("[MoveToHandleState] No valid handle_pose found in userdata.")
#             return "failed"

#         try:
#             original_center = np.array(handle_pose["position"])
#             width = float(handle_pose.get("width", 0.03))  # default if missing

#             # Wrap into PoseStamped from camera frame
#             raw_pose = PoseStamped()
#             raw_pose.header.frame_id = "xtion_depth_optical_frame"
#             raw_pose.header.stamp = rospy.Time.now()
#             raw_pose.pose.position = Point(*original_center)
#             raw_pose.pose.orientation.w = 1.0

#             try:
#                 transformed = self.tf_buffer.transform(
#                     raw_pose, "base_footprint", timeout=rospy.Duration(1.0)
#                 )
#                 center = np.array([
#                     transformed.pose.position.x,
#                     transformed.pose.position.y,
#                     transformed.pose.position.z
#                 ])
#                 rospy.loginfo(f"[MoveToHandleState] Transformed handle center to base_footprint: {center}")
#             except Exception as e:
#                 rospy.logerr(f"[MoveToHandleState] TF transform failed: {e}")
#                 return "failed"

#             # Compute approach position
#             approach_position = center - np.array([self._eef_offset, 0.0, 0.0])

#             # Clamp x and z for safety
#             if approach_position[0] < 0.4:
#                 rospy.logwarn(f"[MoveToHandleState] x={approach_position[0]:.2f} too close. Clamping to 0.4 m.")
#                 approach_position[0] = 0.4
#             approach_position[2] = np.clip(center[2] - 0.05, 0.60, 0.80)

#             # Orientation: slightly pitched forward (about y-axis)
#             qx, qy, qz, qw = quaternion_from_euler(0, 0.4, 0)

#             pose_stamped = PoseStamped()
#             pose_stamped.header.frame_id = "base_footprint"
#             pose_stamped.header.stamp = rospy.Time.now()
#             pose_stamped.pose = Pose(
#                 position=Point(*approach_position),
#                 orientation=Quaternion(qx, qy, qz, qw)
#             )

#             # Debug output
#             rospy.loginfo(f"[MoveToHandleState] Trying to reach: position={approach_position}, quaternion={[qx, qy, qz, qw]}")
#             self._debug_pub.publish(pose_stamped)

#             # Plan and execute
#             self._group.set_start_state_to_current_state()
#             self._group.set_pose_target(pose_stamped)
#             success = self._group.go(wait=True)
#             self._group.stop()
#             self._group.clear_pose_targets()

#             if success:
#                 rospy.loginfo("[MoveToHandleState] Arm successfully moved to handle.")
#                 return "succeeded"
#             else:
#                 rospy.logwarn("[MoveToHandleState] MoveIt failed to plan to handle.")
#                 return "failed"

#         except Exception as e:
#             rospy.logerr(f"[MoveToHandleState] Exception: {e}")
#             return "failed"


# from moveit_commander import MoveGroupCommander
# from geometry_msgs.msg import PoseStamped

# class MoveArmToHandleState(smach.State):
#     def __init__(self, group_name="arm_torso"):
#         smach.State.__init__(self, outcomes=["done", "failed"],
#                              input_keys=["sam_detections"])
#         self.group = MoveGroupCommander(group_name)

#     def execute(self, userdata):
#         if "handle_center" not in userdata.sam_detections:
#             rospy.logerr("[MoveArmToHandleState] No handle_center data.")
#             return "failed"

#         x, y, z = userdata.sam_detections["handle_center"]
#         target_pose = PoseStamped()
#         target_pose.header.frame_id = "xtion_depth_optical_frame"  # 실제 프레임에 맞게
#         target_pose.header.stamp = rospy.Time.now()
#         target_pose.pose.position.x = float(x)
#         target_pose.pose.position.y = float(y)
#         target_pose.pose.position.z = float(z)
#         # 엔드이펙터 오리엔테이션 설정 (예: gripper를 아래로 향하게)
#         target_pose.pose.orientation.w = 1.0

#         self.group.set_pose_target(target_pose)
#         plan = self.group.plan()
#         if not plan or len(plan.joint_trajectory.points) == 0:
#             rospy.logerr("[MoveArmToHandleState] No valid motion plan.")
#             return "failed"

#         success = self.group.go(wait=True)
#         self.group.stop()
#         self.group.clear_pose_targets()

#         return "done" if success else "failed"


    
# import smach
# import numpy as np
# import rospy

# class FilterSafetyDistanceState(smach.State):
#     def __init__(self, max_reach_cm=80.0):
#         smach.State.__init__(
#             self,
#             outcomes=["done", "failed"],
#             input_keys=["sam_detections"],
#             output_keys=["sam_detections"]
#         )
#         self.max_reach_m = max_reach_cm / 100.0  # Convert cm to meters

#     def execute(self, userdata):
#         rospy.loginfo(f"[FilterSafetyDistanceState] Filtering 3D points beyond {self.max_reach_m} m")

#         if "points_3d" not in userdata.sam_detections:
#             rospy.logerr("[FilterSafetyDistanceState] No 3D points found in userdata.")
#             return "failed"

#         points = np.array(userdata.sam_detections["points_3d"])  # shape (N, 3)
#         if points.size == 0:
#             rospy.logwarn("[FilterSafetyDistanceState] No points to filter.")
#             userdata.sam_detections["safe_points"] = []
#             return "done"

#         distances = np.linalg.norm(points, axis=1)  # Euclidean distances
#         safe_points = points[distances > self.max_reach_m]

#         rospy.loginfo(f"[FilterSafetyDistanceState] Found {len(safe_points)} safe points out of {len(points)} total.")

#         # Store filtered safe points back into userdata
#         userdata.sam_detections["safe_points"] = safe_points.tolist()
#         return "done"



# ------------------
# SMACH Main Control
# ------------------
import rospy
import smach
import moveit_commander

def main():
    rospy.init_node("langsam_go_to_point_sm")
    moveit_commander.roscpp_initialize([])

    sm = smach.StateMachine(outcomes=['DONE', 'FAILED'])

    sm.userdata.target_point = Point(x=1.648340, y=0.069282, z=0.925791)  # Fake target

    with sm:
        # smach.StateMachine.add("DETECT_OBJECT",
        #                        LangSamState(prompt="door handle", target_frame="base_footprint"),
        #                        transitions={"succeeded": "GO_TO_POINT", "failed": "FAILED"},
        #                         remapping={'prompt': 'prompt', 'detections': 'detections', 'target_point': 'target_point'})


        smach.StateMachine.add("GO_TO_POINT",
                               GoToOpenPoseState(),
                               transitions={"succeeded": "DONE", "failed": "FAILED"},
                               remapping={"target_point": "target_point"})
        
        # smach.StateMachine.add("DETECT_OBJECT",
        #                        LangSamState(prompt="door handle", target_frame="base_footprint"),
        #                        transitions={"succeeded": "DONE", "failed": "FAILED"},
        #                         remapping={'prompt': 'prompt', 'detections': 'detections', 'target_point': 'target_point'})

    outcome = sm.execute()
    rospy.loginfo(f"State machine finished with outcome: {outcome}")

if __name__ == "__main__":
    main()


        # smach.StateMachine.add(
        #     "POINTCLOUD_EXTRACTION",
        #     PointCloudExtractionState(),
        #     transitions={"done": "PRINT_COUNT", "failed": "FAILED"}
        # )

        # smach.StateMachine.add(
        #     "PRINT_COUNT",
        #     PrintPointCountState(),
        #     transitions={"done": "ESTIMATE_DEPTH", "failed": "FAILED"}        
        # )

        # smach.StateMachine.add(
        #     "ESTIMATE_DEPTH",
        #     EstimateAverageDepthState(),
        #     transitions={"done": "DETECT_HANDLE"}
        # )

        # smach.StateMachine.add(
        #     "DETECT_HANDLE",
        #     DetectHandleState(),
        #     transitions={"succeeded": "DONE", "failed": "FAILED"}
        # )

        # smach.StateMachine.add(
        #     "RAISE_TORSO",
        #     RaiseTorsoState(height=0.15),
        #     transitions={"succeeded": "MOVE_TO_HANDLE", "failed": "FAILED"}
        # )

        # smach.StateMachine.add(
        #     "MOVE_TO_HANDLE",
        #     MoveToHandleState(),
        #     transitions={"succeeded": "DONE", "failed": "FAILED"},
        #     remapping={"handle_pose": "handle_pose"}
        # )


        # smach.StateMachine.add(
        #     "MOVE_ARM_TO_HANDLE",
        #     MoveArmToHandleState(group_name="arm_torso"),
        #     transitions={"done":"DONE", "failed":"FAILED"}
        # )


        


        # smach.StateMachine.add(
        #     "FILTER_SAFETY_DISTANCE",
        #     FilterSafetyDistanceState(max_reach_cm=80.0),  # or whatever Tiago’s arm length is
        #     transitions={"done": "DONE", "failed": "FAILED"}
        # )


# #!/usr/bin/env python

# import sys
# import copy
# import rospy
# import actionlib
# import moveit_commander
# from geometry_msgs.msg import PointStamped, PoseStamped
# from visualization_msgs.msg import Marker
# from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction
# import tf2_ros
# from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
# from moveit_commander.exception import MoveItCommanderException
# from trajectory_msgs.msg import JointTrajectory
# import math

# USE_CLICKED_POINT = True      
# TORSO_LIFT        = 0.25       
# TABLE_THICK       = 0.01      
# TABLE_Z           = 0.41       
# VEL_SCALE         = 0.15     
# ACC_SCALE         = 0.15
# STAMP_DELAY       = 0.25       

# clicked_point = None
# marker_pub    = None

# def clicked_point_cb(msg: PointStamped):
#     """Callback for RViz /clicked_point: store the point and publish a green sphere."""
#     global clicked_point, marker_pub
#     clicked_point = msg
#     m = Marker()
#     m.header = msg.header
#     m.ns, m.id, m.type, m.action = "clicked_points", 0, Marker.SPHERE, Marker.ADD
#     m.pose.position, m.pose.orientation.w = msg.point, 1.0
#     m.scale.x = m.scale.y = m.scale.z = 0.10
#     m.color.r, m.color.a = 1.0, 0.8
#     marker_pub.publish(m)

# def wait_marker_connection():
#     """Block until at least one subscriber is connected to the marker topic."""
#     while marker_pub.get_num_connections() == 0 and not rospy.is_shutdown():
#         rospy.sleep(0.05)

# def publish_arrow(pose: PoseStamped):
#     """Publish a green arrow marker at the given pose."""
#     a = Marker()
#     a.header = pose.header
#     a.ns, a.id, a.type, a.action = "clicked_points", 0, Marker.ARROW, Marker.ADD
#     a.pose = pose.pose
#     a.scale.x = 0.20
#     a.scale.y = a.scale.z = 0.03
#     a.color.g, a.color.a = 1.0, 0.9
#     marker_pub.publish(a)

# def add_time_offset(traj: JointTrajectory, offset: float):
#     """Add a time offset to every point in the JointTrajectory."""
#     for p in traj.points:
#         p.time_from_start += rospy.Duration.from_sec(offset)

# def place_at(x, y, z,
#              arm: moveit_commander.MoveGroupCommander,
#              play: actionlib.SimpleActionClient,
#              gripper_link: str,
#              scene: moveit_commander.PlanningSceneInterface,
#              box_id: str):
#     """
#     Core placement routine:
#     1) Compute the tool-frame goal so that the attached cube's center lands at (x,y,z).
#     2) Plan and execute a motion to that goal.
#     3) Detach the object and open the gripper.
#     Returns True on complete success, False otherwise.
#     """
#     # compute how far below the gripper_link the cube is attached
#     box_offset_z = abs(-0.10)  # same as box_pose.pose.position.z magnitude

#     # tool_goal is the pose we want the gripper_link to reach
#     tool_goal = PoseStamped()
#     tool_goal.header.frame_id = arm.get_planning_frame()
#     tool_goal.pose.position.x = x 
#     tool_goal.pose.position.y = y 
#     tool_goal.pose.position.z = z + box_offset_z
#     # tool_goal.pose.position.z = z 
#     tool_goal.pose.orientation.w = 1.0

#     # 1) Plan and execute move to tool_goal
#     arm.set_position_target(
#         [tool_goal.pose.position.x,
#          tool_goal.pose.position.y,
#          tool_goal.pose.position.z],
#         gripper_link
#     )
#     ok, plan, _, _ = arm.plan()
#     if not ok:
#         rospy.logerr("Planning to placement goal failed")
#         return False
#     if not arm.execute(plan, wait=True):
#         rospy.logerr("Execution to placement goal failed")
#         return False
#     arm.stop()
#     arm.clear_pose_targets()
#     rospy.sleep(0.5)

#     # 2) Detach cube and open gripper
#     scene.remove_attached_object(gripper_link, box_id)
#     open_goal = PlayMotionGoal()
#     open_goal.motion_name   = "open_gripper"
#     open_goal.skip_planning = False
#     play.send_goal_and_wait(open_goal)
#     rospy.sleep(0.5)

#     return True

# def main():
#     global marker_pub, clicked_point

#     rospy.init_node("tiago_place_node")
#     moveit_commander.roscpp_initialize(sys.argv)

#     # set up TF listener and marker publisher
#     tf_buffer   = tf2_ros.Buffer()
#     tf_listener = tf2_ros.TransformListener(tf_buffer)
#     marker_pub  = rospy.Publisher("/click_marker",
#                                  Marker, queue_size=1, latch=True)

#     robot = moveit_commander.RobotCommander()
#     scene = moveit_commander.PlanningSceneInterface()
#     rospy.sleep(1.0)

#     rospy.loginfo("Waiting for planning scene to update …")

#     # 1) Attach a small cube to the gripper as a collision object
#     gripper_link = "gripper_tool_link"
#     box_id       = "held_obj"
#     box_pose = PoseStamped()
#     box_pose.header.frame_id    = gripper_link
#     box_pose.pose.position.z    = -0.10   # cube center is 10cm below tool link
#     box_pose.pose.orientation.w = 1.0
#     scene.add_box(box_id, box_pose, size=(0.04, 0.04, 0.04))
#     rospy.loginfo("Added box %s to scene at %s", box_id, box_pose)
#     rospy.sleep(0.3)
#     scene.attach_box(gripper_link, box_id,
#                      touch_links=robot.get_link_names("gripper"))
#     rospy.loginfo("Attached box %s to gripper link %s", box_id, gripper_link)
#     # 2) Set up play_motion client and do pregrasp + close_gripper
#     play = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
#     play.wait_for_server()
#     rospy.loginfo("Connected to play_motion server")
#     pre = PlayMotionGoal()
#     pre.motion_name   = "deliver_preplace_pose"
#     pre.skip_planning = False
#     play.send_goal_and_wait(pre)
#     rospy.loginfo("Pregrasp motion done, closing gripper …")

#     pre = PlayMotionGoal()
#     pre.motion_name   = "home_to_preplace"
#     pre.skip_planning = False
#     play.send_goal_and_wait(pre)
#     rospy.loginfo("Moved to pre-place pose, closing gripper …")


#     # 3) Define target place pose (can also subscribe to /clicked_point)
#     target = PoseStamped()
#     target.header.frame_id = "base_footprint"
#     target.pose.position.x = 0.6
#     target.pose.position.y = 0.0
#     target.pose.position.z = 0.4
#     target.pose.orientation.w = 1.0

#     wait_marker_connection()
#     publish_arrow(target)

#     if USE_CLICKED_POINT:
#         rospy.Subscriber("/clicked_point", PointStamped, clicked_point_cb)
#         rospy.loginfo("Waiting for RViz click …")
#         while clicked_point is None and not rospy.is_shutdown():
#             rospy.sleep(0.05)
#         target.pose.position = clicked_point.point
#         target.header        = clicked_point.header
#         publish_arrow(target)

#     # 4) Initialize MoveIt commander for the arm
#     arm = moveit_commander.MoveGroupCommander("arm_torso")
#     arm.set_end_effector_link(gripper_link)
#     arm.set_pose_reference_frame("base_footprint")
#     arm.set_planning_time(30.0)
#     arm.set_num_planning_attempts(10)
#     arm.set_max_velocity_scaling_factor(VEL_SCALE)
#     arm.set_max_acceleration_scaling_factor(ACC_SCALE)
#     arm.set_goal_tolerance(0.01)
#     arm.set_goal_orientation_tolerance(math.pi)

#     # wait for current joint states
#     while len(arm.get_current_joint_values()) != len(arm.get_active_joints()) \
#           and not rospy.is_shutdown():
#         rospy.sleep(0.05)

#     # 5) Raise the torso
#     arm.set_joint_value_target({'torso_lift_joint': TORSO_LIFT})
#     arm.go(wait=True)
#     arm.stop(); arm.clear_pose_targets()

#     # 6) Add the table to the planning scene
#     # table_pose = PoseStamped()
#     # table_pose.header.frame_id = "base_footprint"
#     # table_pose.pose.position.x = 0.8
#     # table_pose.pose.position.z = TABLE_Z
#     # table_pose.pose.orientation.w = 1.0
#     # scene.add_box("table", table_pose, size=(1.0, 1.2, TABLE_THICK))
#     # arm.set_support_surface_name("table")


#     # transform target into the arm's planning frame if needed
#     planning_frame = arm.get_planning_frame()
#     if target.header.frame_id != planning_frame:
#         tr = tf_buffer.lookup_transform(planning_frame,
#                                         target.header.frame_id,
#                                         rospy.Time(0),
#                                         rospy.Duration(1.0))
#         target = do_transform_pose(target, tr)

#     # 7) Call the placement function
#     success = place_at(
#         target.pose.position.x,
#         target.pose.position.y,
#         target.pose.position.z,
#         arm=arm,
#         play=play,
#         gripper_link=gripper_link,
#         scene=scene,
#         box_id=box_id
#     )
#     rospy.loginfo("Place operation %s", "SUCCEEDED" if success else "FAILED")

#     # 8) Return to home
#     home = PlayMotionGoal()
#     home.motion_name   = "home"
#     home.skip_planning = False
#     play.send_goal_and_wait(home)

#     moveit_commander.roscpp_shutdown()

# if __name__ == "__main__":
#     main()

