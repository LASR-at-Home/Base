#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import numpy as np
import ros_numpy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image as SensorImage, PointCloud2
from lasr_vision_msgs.srv import LangSam, LangSamRequest
import sensor_msgs.point_cloud2 as pc2

# --------------------------
# LangSAM State (gets mask)
# --------------------------
import rospy
import smach
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as SensorImage
from lasr_vision_msgs.srv import LangSam, LangSamRequest


"""
1. LangSam
- return mask
- if not mask return a box cutting out the edges or side
2. getDepth
- collect Depth on mask
- return Depth
* RotateDepth
- face with door
3. analyseDepth
- longDepth > set safety line
- shortDepth > handle
4. analyseHandle > width, height

pos_1(x: shortDepth - 0.01, y: point of shortest depth starts close to robot +- width/2 (+ assume right, - assume left..?), shortDepth starts + height/2)

if width shorter: handle shape []
if height shorter: handle shape =

Wide open handle
pos_2(x: shortDepth + (longDepth - shortDepth)/2, ", ")

Close handle according to width

4. Pull, Push, Rotate logic for door openning 
"""


class LangSamState(smach.State):
    def __init__(self, prompt: str, rgb_topic="/xtion/rgb/image_raw"):
        smach.State.__init__(self, outcomes=["succeeded", "failed"], output_keys=["sam_detections"])
        self.prompt = prompt
        self.rgb_topic = rgb_topic

        self.mask_pub = rospy.Publisher("/langsam/mask_image", SensorImage, queue_size=1)
        self.bridge = CvBridge()

        rospy.loginfo("[LangSamState] Waiting for LangSAM service...")
        rospy.wait_for_service("/lasr_vision/lang_sam", timeout=5)
        self._client = rospy.ServiceProxy("/lasr_vision/lang_sam", LangSam)
        rospy.loginfo("[LangSamState] Connected to LangSAM service.")

    def execute(self, userdata):
        rospy.loginfo(f"[LangSamState] Waiting for image on topic: {self.rgb_topic}")
        try:
            image_msg = rospy.wait_for_message(self.rgb_topic, SensorImage, timeout=5.0)
        except rospy.ROSException:
            rospy.logerr("[LangSamState] Timeout while waiting for RGB image.")
            return "failed"

        req = LangSamRequest()
        req.prompt = self.prompt
        req.image_raw = image_msg

        try:
            resp = self._client(req)

            if not resp.detections:
                return "failed"

            rospy.loginfo(f"[LangSamState] LangSAM returned {len(resp.detections)} detections.")

            best_idx = np.argmax([d.seg_mask_score for d in resp.detections])
            det = resp.detections[best_idx]

            width = image_msg.width
            height = image_msg.height

            # Convert flat mask list to 2D binary mask
            flat_mask = np.array(det.seg_mask, dtype=np.uint8)
            mask_np = flat_mask.reshape((height, width))

            # Filter: Reject mask if too small or touches image borders
            mask_area = np.count_nonzero(mask_np)
            edge_threshold = 10  # pixels
            margin_mask = mask_np[
                edge_threshold:-edge_threshold,
                edge_threshold:-edge_threshold
            ]
            interior_area = np.count_nonzero(margin_mask)
            interior_ratio = interior_area / (mask_area + 1e-6)

            if mask_area < 5000 or interior_ratio < 0.7:
                rospy.logwarn(f"[LangSamState] Mask too small or touches image edge. Area={mask_area}, Interior Ratio={interior_ratio:.2f}")
                return "failed"

            # Publish mask for visualization
            vis_mask = mask_np * 255
            mask_msg = self.bridge.cv2_to_imgmsg(vis_mask, encoding="mono8")
            mask_msg.header = image_msg.header
            self.mask_pub.publish(mask_msg)

            # Pass result to userdata
            userdata.sam_detections = {
                "mask": mask_np,
                "image_header": image_msg.header
            }

            return "succeeded"

        except rospy.ServiceException as e:
            rospy.logerr(f"LangSAM service call failed: {e}")
            return "failed"


# ------------------------------------
# Helper: Extract 3D points from cloud
# ------------------------------------
def extract_points_from_pointcloud(mask_np, cloud_msg):
    height = cloud_msg.height
    width = cloud_msg.width

    if mask_np.shape != (height, width):
        rospy.logwarn("[extract_points_from_pointcloud] Mask shape mismatch.")
        return []

    rospy.loginfo(f"[DEBUG] PointCloud size: {width}x{height}")
    rospy.loginfo(f"[DEBUG] Mask size: {mask_np.shape}")

    cloud_iter = pc2.read_points(cloud_msg, skip_nans=False, field_names=("x", "y", "z"))
    cloud_array = np.array(list(cloud_iter), dtype=np.float32).reshape((height, width, 3))

    selected_points = cloud_array[mask_np.astype(bool)]
    selected_points = selected_points[~np.isnan(selected_points).any(axis=1)]

    rospy.loginfo(f"[DEBUG] Masked pixels: {np.count_nonzero(mask_np)}, Valid 3D points: {len(selected_points)}")

    return selected_points.tolist()


# -------------------------------
# State: Extract 3D points from cloud
# -------------------------------
class PointCloudExtractionState(smach.State):
    def __init__(self, cloud_topic="/xtion/depth_pointsros"):
        smach.State.__init__(self, outcomes=["done", "failed"],
                             input_keys=["sam_detections"],
                             output_keys=["sam_detections"])
        self.cloud_topic = cloud_topic

    def execute(self, userdata):
        rospy.loginfo("[PointCloudExtractionState] Waiting for organized point cloud...")
        try:
            cloud_msg = rospy.wait_for_message(self.cloud_topic, PointCloud2, timeout=5.0)
        except rospy.ROSException:
            rospy.logerr("[PointCloudExtractionState] Failed to get PointCloud2.")
            return "failed"

        mask = userdata.sam_detections["mask"]
        points_3d = extract_points_from_pointcloud(mask, cloud_msg)

        rospy.loginfo(f"[PointCloudExtractionState] Extracted {len(points_3d)} valid 3D points from point cloud.")
        userdata.sam_detections["points_3d"] = points_3d
        return "done"


# ----------------------
# Final print/log state
# ----------------------
class PrintPointCountState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done", "failed"], input_keys=["sam_detections"])

    def execute(self, userdata):
        points = userdata.sam_detections.get("points_3d", [])
        rospy.loginfo(f"[PrintPointCountState] 3D points count: {len(points)}")
        return "done"
    

class EstimateAverageDepthState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done"], input_keys=["sam_detections"], output_keys=["sam_detections"])

    def execute(self, userdata):
        points = userdata.sam_detections.get("points_3d", [])
        if not points:
            rospy.logwarn("[EstimateAverageDepthState] No 3D points to process.")
            userdata.sam_detections["average_depth"] = None
            return "done"

        z_values = [p[2] for p in points if not np.isnan(p[2])]
        if not z_values:
            rospy.logwarn("[EstimateAverageDepthState] All depth values are NaN.")
            userdata.sam_detections["average_depth"] = None
            return "done"

        avg_depth = np.mean(z_values)
        min_depth = np.min(z_values)
        max_depth = np.max(z_values)

        rospy.loginfo(f"[EstimateAverageDepthState] Avg depth: {avg_depth:.3f} m, Min: {min_depth:.3f} m, Max: {max_depth:.3f} m")

        userdata.sam_detections["average_depth"] = avg_depth
        userdata.sam_detections["min_depth"] = min_depth
        userdata.sam_detections["max_depth"] = max_depth
        return "done"
    
import rospy
import smach
import numpy as np
from sklearn.decomposition import PCA
from sklearn.cluster import DBSCAN
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_matrix

class DetectHandleState(smach.State):
    def __init__(self, use_dbscan=True):
        smach.State.__init__(
            self,
            outcomes=["done", "failed"],
            input_keys=["sam_detections"],
            output_keys=["handle_pose"]
        )
        self.use_dbscan = use_dbscan

    def execute(self, userdata):
        points = userdata.sam_detections.get("points_3d", [])
        if not points:
            rospy.logwarn("[DetectHandleState] No 3D points provided.")
            return "failed"

        points_np = np.array(points)

        # Step 1: DBSCAN clustering (optional)
        if self.use_dbscan:
            try:
                clustering = DBSCAN(eps=0.02, min_samples=10).fit(points_np)
                labels = clustering.labels_

                # Filter out noise
                mask = labels != -1
                labels = labels[mask]
                clustered_points = points_np[mask]

                # Heuristic: find cluster closest to robot (min z)
                cluster_ids = np.unique(labels)
                best_cluster_id = None
                best_min_z = float('inf')
                for cid in cluster_ids:
                    cluster_pts = clustered_points[labels == cid]
                    min_z = np.min(cluster_pts[:, 2])
                    if min_z < best_min_z:
                        best_min_z = min_z
                        best_cluster_id = cid

                handle_points = clustered_points[labels == best_cluster_id]
                rospy.loginfo(f"[DetectHandleState] Selected cluster {best_cluster_id} with {len(handle_points)} points.")
            except Exception as e:
                rospy.logerr(f"[DetectHandleState] DBSCAN error: {e}")
                return "failed"
        else:
            handle_points = points_np

        if len(handle_points) < 10:
            rospy.logwarn("[DetectHandleState] Not enough points to estimate handle.")
            return "failed"

        # Step 2: PCA for orientation and size
        try:
            pca = PCA(n_components=3)
            pca.fit(handle_points)
            center = pca.mean_

            # Project to PCA space
            proj = pca.transform(handle_points)
            width = proj[:, 1].ptp()  # range along axis 1
            height = proj[:, 2].ptp()  # range along axis 2
            handle_width = min(width, height)

            # Orientation from PCA axes
            x_axis = pca.components_[0]
            y_axis = pca.components_[1]
            z_axis = np.cross(x_axis, y_axis)

            # Rotation matrix to quaternion
            rot = np.eye(4)
            rot[:3, 0] = x_axis
            rot[:3, 1] = y_axis
            rot[:3, 2] = z_axis
            qx, qy, qz, qw = quaternion_from_matrix(rot)

            rospy.loginfo(f"[DetectHandleState] Handle center: {center}")
            rospy.loginfo(f"[DetectHandleState] Width: {width:.3f}, Height: {height:.3f}")
            rospy.loginfo(f"[DetectHandleState] Orientation (quat): {[qx, qy, qz, qw]}")

            userdata.handle_pose = {
                "position": center.tolist(),
                "quaternion": [qx, qy, qz, qw],
                "width": float(handle_width)
            }
            return "done"

        except Exception as e:
            rospy.logerr(f"[DetectHandleState] PCA error: {e}")
            return "failed"


    
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
def main():
    rospy.init_node("lang_sam_pointcloud_pipeline")

    sm = smach.StateMachine(outcomes=["DONE", "FAILED"])
    with sm:
        smach.StateMachine.add(
            "SEGMENT_DOOR",
            LangSamState(prompt="a door"),
            transitions={"succeeded": "POINTCLOUD_EXTRACTION", "failed": "FAILED"}
        )

        smach.StateMachine.add(
            "POINTCLOUD_EXTRACTION",
            PointCloudExtractionState(),
            transitions={"done": "PRINT_COUNT", "failed": "FAILED"}
        )

        smach.StateMachine.add(
            "PRINT_COUNT",
            PrintPointCountState(),
            transitions={"done": "ESTIMATE_DEPTH", "failed": "FAILED"}        
        )

        smach.StateMachine.add(
            "ESTIMATE_DEPTH",
            EstimateAverageDepthState(),
            transitions={"done": "DETECT_HANDLE"}
        )

        smach.StateMachine.add(
            "DETECT_HANDLE",
            DetectHandleState(),
            transitions={"done": "DONE", "failed": "FAILED"}
        )

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




    outcome = sm.execute()
    rospy.loginfo(f"[StateMachine] Final Outcome: {outcome}")


if __name__ == "__main__":
    main()


#!/usr/bin/env python

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

clicked_point = None
marker_pub    = None

def clicked_point_cb(msg: PointStamped):
    """Callback for RViz /clicked_point: store the point and publish a green sphere."""
    global clicked_point, marker_pub
    clicked_point = msg
    m = Marker()
    m.header = msg.header
    m.ns, m.id, m.type, m.action = "clicked_points", 0, Marker.SPHERE, Marker.ADD
    m.pose.position, m.pose.orientation.w = msg.point, 1.0
    m.scale.x = m.scale.y = m.scale.z = 0.10
    m.color.r, m.color.a = 1.0, 0.8
    marker_pub.publish(m)

def wait_marker_connection():
    """Block until at least one subscriber is connected to the marker topic."""
    while marker_pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.sleep(0.05)

def publish_arrow(pose: PoseStamped):
    """Publish a green arrow marker at the given pose."""
    a = Marker()
    a.header = pose.header
    a.ns, a.id, a.type, a.action = "clicked_points", 0, Marker.ARROW, Marker.ADD
    a.pose = pose.pose
    a.scale.x = 0.20
    a.scale.y = a.scale.z = 0.03
    a.color.g, a.color.a = 1.0, 0.9
    marker_pub.publish(a)

def add_time_offset(traj: JointTrajectory, offset: float):
    """Add a time offset to every point in the JointTrajectory."""
    for p in traj.points:
        p.time_from_start += rospy.Duration.from_sec(offset)

def place_at(x, y, z,
             arm: moveit_commander.MoveGroupCommander,
             play: actionlib.SimpleActionClient,
             gripper_link: str,
             scene: moveit_commander.PlanningSceneInterface,
             box_id: str):
    """
    Core placement routine:
    1) Compute the tool-frame goal so that the attached cube's center lands at (x,y,z).
    2) Plan and execute a motion to that goal.
    3) Detach the object and open the gripper.
    Returns True on complete success, False otherwise.
    """
    # compute how far below the gripper_link the cube is attached
    box_offset_z = abs(-0.10)  # same as box_pose.pose.position.z magnitude

    # tool_goal is the pose we want the gripper_link to reach
    tool_goal = PoseStamped()
    tool_goal.header.frame_id = arm.get_planning_frame()
    tool_goal.pose.position.x = x 
    tool_goal.pose.position.y = y 
    tool_goal.pose.position.z = z + box_offset_z
    # tool_goal.pose.position.z = z 
    tool_goal.pose.orientation.w = 1.0

    # 1) Plan and execute move to tool_goal
    arm.set_position_target(
        [tool_goal.pose.position.x,
         tool_goal.pose.position.y,
         tool_goal.pose.position.z],
        gripper_link
    )
    ok, plan, _, _ = arm.plan()
    if not ok:
        rospy.logerr("Planning to placement goal failed")
        return False
    if not arm.execute(plan, wait=True):
        rospy.logerr("Execution to placement goal failed")
        return False
    arm.stop()
    arm.clear_pose_targets()
    rospy.sleep(0.5)

    # 2) Detach cube and open gripper
    scene.remove_attached_object(gripper_link, box_id)
    open_goal = PlayMotionGoal()
    open_goal.motion_name   = "open_gripper"
    open_goal.skip_planning = False
    play.send_goal_and_wait(open_goal)
    rospy.sleep(0.5)

    return True

def main():
    global marker_pub, clicked_point

    rospy.init_node("tiago_place_node")
    moveit_commander.roscpp_initialize(sys.argv)

    # set up TF listener and marker publisher
    tf_buffer   = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    marker_pub  = rospy.Publisher("/click_marker",
                                 Marker, queue_size=1, latch=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1.0)

    rospy.loginfo("Waiting for planning scene to update …")

    # 1) Attach a small cube to the gripper as a collision object
    gripper_link = "gripper_tool_link"
    box_id       = "held_obj"
    box_pose = PoseStamped()
    box_pose.header.frame_id    = gripper_link
    box_pose.pose.position.z    = -0.10   # cube center is 10cm below tool link
    box_pose.pose.orientation.w = 1.0
    scene.add_box(box_id, box_pose, size=(0.04, 0.04, 0.04))
    rospy.loginfo("Added box %s to scene at %s", box_id, box_pose)
    rospy.sleep(0.3)
    scene.attach_box(gripper_link, box_id,
                     touch_links=robot.get_link_names("gripper"))
    rospy.loginfo("Attached box %s to gripper link %s", box_id, gripper_link)
    # 2) Set up play_motion client and do pregrasp + close_gripper
    play = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
    play.wait_for_server()
    rospy.loginfo("Connected to play_motion server")
    pre = PlayMotionGoal()
    pre.motion_name   = "deliver_preplace_pose"
    pre.skip_planning = False
    play.send_goal_and_wait(pre)
    rospy.loginfo("Pregrasp motion done, closing gripper …")

    pre = PlayMotionGoal()
    pre.motion_name   = "home_to_preplace"
    pre.skip_planning = False
    play.send_goal_and_wait(pre)
    rospy.loginfo("Moved to pre-place pose, closing gripper …")


    # 3) Define target place pose (can also subscribe to /clicked_point)
    target = PoseStamped()
    target.header.frame_id = "base_footprint"
    target.pose.position.x = 0.6
    target.pose.position.y = 0.0
    target.pose.position.z = 0.4
    target.pose.orientation.w = 1.0

    wait_marker_connection()
    publish_arrow(target)

    if USE_CLICKED_POINT:
        rospy.Subscriber("/clicked_point", PointStamped, clicked_point_cb)
        rospy.loginfo("Waiting for RViz click …")
        while clicked_point is None and not rospy.is_shutdown():
            rospy.sleep(0.05)
        target.pose.position = clicked_point.point
        target.header        = clicked_point.header
        publish_arrow(target)

    # 4) Initialize MoveIt commander for the arm
    arm = moveit_commander.MoveGroupCommander("arm_torso")
    arm.set_end_effector_link(gripper_link)
    arm.set_pose_reference_frame("base_footprint")
    arm.set_planning_time(30.0)
    arm.set_num_planning_attempts(10)
    arm.set_max_velocity_scaling_factor(VEL_SCALE)
    arm.set_max_acceleration_scaling_factor(ACC_SCALE)
    arm.set_goal_tolerance(0.01)
    arm.set_goal_orientation_tolerance(math.pi)

    # wait for current joint states
    while len(arm.get_current_joint_values()) != len(arm.get_active_joints()) \
          and not rospy.is_shutdown():
        rospy.sleep(0.05)

    # 5) Raise the torso
    arm.set_joint_value_target({'torso_lift_joint': TORSO_LIFT})
    arm.go(wait=True)
    arm.stop(); arm.clear_pose_targets()

    # 6) Add the table to the planning scene
    # table_pose = PoseStamped()
    # table_pose.header.frame_id = "base_footprint"
    # table_pose.pose.position.x = 0.8
    # table_pose.pose.position.z = TABLE_Z
    # table_pose.pose.orientation.w = 1.0
    # scene.add_box("table", table_pose, size=(1.0, 1.2, TABLE_THICK))
    # arm.set_support_surface_name("table")


    # transform target into the arm's planning frame if needed
    planning_frame = arm.get_planning_frame()
    if target.header.frame_id != planning_frame:
        tr = tf_buffer.lookup_transform(planning_frame,
                                        target.header.frame_id,
                                        rospy.Time(0),
                                        rospy.Duration(1.0))
        target = do_transform_pose(target, tr)

    # 7) Call the placement function
    success = place_at(
        target.pose.position.x,
        target.pose.position.y,
        target.pose.position.z,
        arm=arm,
        play=play,
        gripper_link=gripper_link,
        scene=scene,
        box_id=box_id
    )
    rospy.loginfo("Place operation %s", "SUCCEEDED" if success else "FAILED")

    # 8) Return to home
    home = PlayMotionGoal()
    home.motion_name   = "home"
    home.skip_planning = False
    play.send_goal_and_wait(home)

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()

