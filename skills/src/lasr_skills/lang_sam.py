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
            rospy.loginfo(f"[LangSamState] LangSAM returned {len(resp.detections)} detections.")

            if not resp.detections:
                return "failed"

            best_idx = np.argmax([d.seg_mask_score for d in resp.detections])
            det = resp.detections[best_idx]

            width = image_msg.width
            height = image_msg.height

            # FIX: Convert flat mask list to 2D binary mask
            flat_mask = np.array(det.seg_mask, dtype=np.uint8)
            mask_np = flat_mask.reshape((height, width))

            # Publish mask for visualization (scaled to 255)
            vis_mask = mask_np * 255
            mask_msg = self.bridge.cv2_to_imgmsg(vis_mask, encoding="mono8")
            mask_msg.header = image_msg.header
            self.mask_pub.publish(mask_msg)

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
        return "done"
    

import numpy as np
from sklearn.decomposition import PCA
import rospy
import smach


class DetectHandleState(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["done", "failed"],
            input_keys=["sam_detections"],
            output_keys=["handle_pose"]
        )

    def execute(self, userdata):
        points = userdata.sam_detections.get("points_3d", [])

        if not points:
            rospy.logwarn("[DetectHandleState] No 3D points provided.")
            return "failed"

        points_np = np.array(points)

        # Step 1: Cluster analysis (simplified for now: just use all points)
        # You can apply DBSCAN or filtering later

        # Step 2: Estimate depth profile along width
        # Project to xz-plane to find width and depth changes
        x_vals = points_np[:, 0]
        z_vals = points_np[:, 2]

        # Bin by x to analyze depth variance (simulate depth edge where handle is)
        try:
            hist_bins = 100
            bin_indices = np.digitize(x_vals, np.linspace(x_vals.min(), x_vals.max(), hist_bins))

            mean_depth_per_bin = np.zeros(hist_bins)
            for i in range(1, hist_bins + 1):
                bin_points = z_vals[bin_indices == i]
                mean_depth_per_bin[i - 1] = np.nanmean(bin_points) if len(bin_points) > 0 else np.nan

            # Step 3: Detect minimum depth (assumed handle location)
            min_depth_idx = np.nanargmin(mean_depth_per_bin)
            handle_x_min = np.linspace(x_vals.min(), x_vals.max(), hist_bins)[min_depth_idx]

            # Extract handle point cluster near that x position
            handle_mask = np.abs(x_vals - handle_x_min) < 0.02  # 2 cm slice
            handle_points = points_np[handle_mask]

            if len(handle_points) < 10:
                rospy.logwarn("[DetectHandleState] Not enough points to estimate handle.")
                return "failed"

            # Step 4: Fit PCA to find precise width & center
            pca = PCA(n_components=3)
            pca.fit(handle_points)
            center = pca.mean_  # (x, y, z)
            lengths = 2 * np.sqrt(pca.explained_variance_)
            width = min(lengths[:2])  # Smaller of horizontal axes

            rospy.loginfo(f"[DetectHandleState] Handle width: {width:.3f} m")
            rospy.loginfo(f"[DetectHandleState] Handle center position: {center}")

            userdata.handle_pose = {
                "position": center.tolist(),
                "width": float(width)
            }
            return "done"

        except Exception as e:
            rospy.logerr(f"[DetectHandleState] Error: {e}")
            return "failed"
        

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
            transitions={"done": "done", "failed": "FAILED"}
        )

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
