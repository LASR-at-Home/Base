from typing import List, Tuple, Optional

import rospy
import rospkg
import actionlib
import open3d as o3d
import tf.transformations as tf
import numpy as np
import os
import subprocess
from copy import deepcopy

from moveit_commander import MoveGroupCommander

from geometry_msgs.msg import (
    TransformStamped,
    Vector3Stamped,
    Pose,
    PoseStamped,
    Point,
    Quaternion,
)
from std_srvs.srv import Empty
from std_msgs.msg import Header
from control_msgs.msg import JointTrajectoryControllerState
import tf2_geometry_msgs
import tf2_ros

from lasr_manipulation_msgs.msg import PickAction, PickGoal, PickResult
from lasr_manipulation_msgs.srv import AllowCollisionsWithObj, AttachObjectToGripper
from visualization_msgs.msg import Marker, MarkerArray

from pal_startup_msgs.srv import (
    StartupStart,
    StartupStartRequest,
    StartupStop,
    StartupStopRequest,
)

CALIBRATION_OFFSET_X: float = 0.0
CALIBRATION_OFFSET_Y: float = -0.025
CALIBRATION_OFFSET_Z: float = 0.0


class PickServer:
    """
    An action server for picking objects.
    """

    # Meshes
    _mesh_dir: str = os.path.join(
        rospkg.RosPack().get_path("lasr_manipulation_pipeline"), "meshes"
    )

    # GPD
    _gpd_binary_path: str = "/opt/gpd/build/detect_grasps"
    _gpd_config_path: str = os.path.join(
        rospkg.RosPack().get_path("lasr_manipulation_pipeline"), "config", "tiago.cfg"
    )
    _gpd_pcd_path: str = "/tmp/gpd.pcd"
    _gpd_output_file: str = "grasps.txt"

    # Action server
    _pick_server: actionlib.SimpleActionServer

    # MoveIt
    _move_group: MoveGroupCommander
    _close_gripper: rospy.ServiceProxy

    # Planning scene services
    _allow_collisions_with_obj: rospy.ServiceProxy
    _attach_object_to_gripper: rospy.ServiceProxy

    # Tf
    _tf_buffer: tf2_ros.Buffer
    _tf_listener: tf2_ros.TransformListener

    # Debug publisher
    _grasp_markers_pub: rospy.Publisher

    def __init__(self) -> None:
        self._pick_server = actionlib.SimpleActionServer(
            "/lasr_manipulation/pick",
            PickAction,
            execute_cb=self._pick,
            auto_start=False,
        )

        # Setup Move Group
        self._move_group = MoveGroupCommander("arm_torso")
        self._move_group.set_planner_id("RRTConnectkConfigDefault")
        self._move_group.allow_replanning(True)
        self._move_group.allow_looking(True)
        self._move_group.set_planning_time(30)
        self._move_group.set_num_planning_attempts(30)
        self._move_group.set_max_velocity_scaling_factor(0.5)

        self._close_gripper = rospy.ServiceProxy(
            "/parallel_gripper_controller/grasp", Empty
        )
        self._close_gripper.wait_for_service()

        self._allow_collisions_with_obj = rospy.ServiceProxy(
            "/lasr_manipulation_planning_scene/allow_collisions_with_obj",
            AllowCollisionsWithObj,
        )
        self._allow_collisions_with_obj.wait_for_service()
        self._attach_object_to_gripper = rospy.ServiceProxy(
            "/lasr_manipulation_planning_scene/attach_object_to_gripper",
            AttachObjectToGripper,
        )
        self._attach_object_to_gripper.wait_for_service()

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._enable_head_manager = rospy.ServiceProxy(
            "/pal_startup_control/start", StartupStart
        )
        self._disable_head_manager = rospy.ServiceProxy(
            "/pal_startup_control/start", StartupStop
        )

        self._grasp_markers_pub = rospy.Publisher(
            "/grasp_markers", MarkerArray, queue_size=10, latch=True
        )

        self._pick_server.start()

        rospy.loginfo("/lasr_manipulation/pick has started!")

    def _pick(self, goal: PickGoal) -> None:
        object_id: str = goal.object_id
        mesh_name: str = goal.mesh_name
        transform: TransformStamped = goal.transform
        scale: Vector3Stamped = goal.scale

        if self._pick_server.is_preempt_requested():
            rospy.loginfo("Pick goal preempted before starting.")
            self._pick_server.set_preempted()
            return

        # Generate grasps
        grasps, scores = self._generate_grasps(mesh_name, transform, scale)

        if self._pick_server.is_preempt_requested():
            rospy.loginfo("Pick goal preempted after grasp generation.")
            self._pick_server.set_preempted()
            return

        # If no grasps are outputted, finish.
        if not grasps:
            result = PickResult(success=False)
            self._pick_server.set_aborted(result)
            return

        self._publish_grasp_poses([g.pose for g in grasps], grasps[0].header.frame_id)

        # Clear any existing pose targets
        self._move_group.clear_pose_targets()

        # Execute pre-grasps
        rospy.loginfo("Dispatching pre-grasps to MoveIt for execution...")
        rospy.loginfo(f"Setting frame ID to {grasps[0].header.frame_id}")
        self._move_group.set_pose_reference_frame(grasps[0].header.frame_id)
        self._move_group.set_pose_targets(
            [g.pose for g in grasps], "gripper_grasping_frame"
        )

        if self._pick_server.is_preempt_requested():
            rospy.loginfo("Pick goal preempted before pre-grasp execution.")
            self._pick_server.set_preempted()
            return

        success = self._move_group.go(wait=True)
        if not success:
            rospy.logwarn("MoveIt failed to execute pre-grasps. Aborting.")
            result = PickResult(success=False)
            self._pick_server.set_aborted(result)
            return

        rospy.loginfo("Reached pre-grasp pose")

        if self._pick_server.is_preempt_requested():
            rospy.loginfo("Pick goal preempted after reaching pre-grasp.")
            self._pick_server.set_preempted()
            return

        # Allow the end-effector to touch the object
        self._allow_collisions_with_obj(object_id)

        self._move_group.set_support_surface_name("table")

        # Get final grasp pose
        final_grasp_pose = self._get_final_grasp_pose()
        rospy.loginfo("Got final grasp pose")
        self._publish_grasp_poses(
            [final_grasp_pose.pose], final_grasp_pose.header.frame_id
        )

        self._move_group.clear_pose_targets()

        self._move_group.set_pose_reference_frame(final_grasp_pose.header.frame_id)
        self._move_group.set_pose_target(
            final_grasp_pose.pose, "gripper_grasping_frame"
        )

        if self._pick_server.is_preempt_requested():
            rospy.loginfo("Pick goal preempted before final grasp execution.")
            self._pick_server.set_preempted()
            return

        success = self._move_group.go(wait=True)
        if not success:
            rospy.loginfo("MoveIt failed to execute final grasp. Aborting.")
            result = PickResult(success=False)
            self._pick_server.set_aborted(result)
            return

        rospy.loginfo("Reached final grasp pose")

        if self._pick_server.is_preempt_requested():
            rospy.loginfo("Pick goal preempted before closing gripper.")
            self._pick_server.set_preempted()
            return

        # Close gripper
        self._close_gripper()
        rospy.loginfo("Closed gripper")

        if self._pick_server.is_preempt_requested():
            rospy.loginfo("Pick goal preempted after closing gripper.")
            self._pick_server.set_preempted()
            return

        # Check if grasp was successful
        success = not self._eef_is_fully_closed()
        if not success:
            rospy.loginfo(
                "End-effector is fully closed, so grasp likely failed. Aborting."
            )
            result = PickResult(success=False)
            self._pick_server.set_aborted(result)
            return

        rospy.loginfo("Grasp was successful")

        self._attach_object_to_gripper(object_id)
        rospy.loginfo(f"Attached {object_id} to gripper")

        # Get post grasp pose
        post_grasp_pose = self._get_post_grasp_pose()
        rospy.loginfo("Got pose grasp pose")
        self._publish_grasp_poses(
            [post_grasp_pose.pose], post_grasp_pose.header.frame_id
        )

        self._move_group.clear_pose_targets()

        self._move_group.set_pose_reference_frame(post_grasp_pose.header.frame_id)
        self._move_group.set_pose_target(post_grasp_pose.pose, "gripper_grasping_frame")

        if self._pick_server.is_preempt_requested():
            rospy.loginfo("Pick goal preempted before post grasp execution.")
            self._pick_server.set_preempted()
            return

        success = self._move_group.go(wait=True)
        if not success:
            rospy.loginfo("MoveIt failed to execute post grasp. Aborting.")
            result = PickResult(success=False)
            self._pick_server.set_aborted(result)
            return

        result = PickResult(success=True, grasp_pose=final_grasp_pose)
        self._pick_server.set_succeeded(result)
        self._enable_head_manager(StartupStartRequest("head_manager", ""))
        rospy.loginfo("Pick was successful")

    def _publish_grasp_poses(self, poses: List[Pose], frame_id: str = "base_footprint"):
        marker_array = MarkerArray()
        for idx, pose in enumerate(poses):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "grasp_poses"
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.12  # shaft length
            marker.scale.x = 0.09  # shaft length
            marker.scale.y = 0.02  # shaft diameter
            marker.scale.z = 0.02  # head diameter
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(marker)
        self._grasp_markers_pub.publish(marker_array)
        rospy.loginfo(f"Published {len(poses)} grasp pose markers.")

    def _generate_grasps(
        self, mesh_name: str, transform: TransformStamped, scale: Vector3Stamped
    ) -> Tuple[List[PoseStamped], List[float]]:
        """
        Load a mesh, apply a transform and scale, generate grasps using GPD, and return sorted grasps with scores.
        """
        mesh = self._load_and_transform_mesh(mesh_name, transform, scale)
        if mesh is None:
            return [], []

        self._write_point_cloud_for_gpd(mesh)

        success = self._run_gpd()
        if not success:
            return [], []

        grasps, scores = self._read_and_parse_gpd_output(transform.header.frame_id)

        if not grasps:
            return [], []

        sorted_pairs = sorted(zip(grasps, scores), key=lambda x: x[1], reverse=True)
        grasps, scores = zip(*sorted_pairs)

        grasps, scores = self._postprocess_grasps(mesh, grasps, scores)

        return list(grasps), list(scores)

    def _load_and_transform_mesh(
        self, mesh_name: str, transform: TransformStamped, scale: Vector3Stamped
    ) -> Optional[o3d.geometry.PointCloud]:
        """
        Loads the mesh, applies a transformation and scaling, and returns the processed point cloud.
        """
        mesh_path = os.path.join(self._mesh_dir, f"{mesh_name}.ply")
        mesh = o3d.io.read_triangle_mesh(mesh_path)

        if mesh.is_empty():
            rospy.logwarn("Mesh is empty, so no grasps will be generated.")
            return None

        pcd = mesh.sample_points_poisson_disk(number_of_points=10000, init_factor=5)
        rospy.loginfo(
            f"Read a mesh with {np.asarray(pcd.points).shape[0]} points."
        )  # TODO: do we actually want this?

        trans = transform.transform.translation
        quat = transform.transform.rotation
        rot_mat = tf.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])[:3, :3]

        sx, sy, sz = scale.vector.x, scale.vector.y, scale.vector.z
        scale_mat = np.diag([sx, sy, sz])

        rs_mat = np.dot(rot_mat, scale_mat)

        T = np.eye(4)
        T[:3, :3] = rs_mat
        T[:3, 3] = [trans.x, trans.y, trans.z]

        rospy.loginfo(f"Constructed non-rigid transformation matrix:\n{T}")

        pcd.transform(T)

        return pcd

    def _write_point_cloud_for_gpd(self, pcd: o3d.geometry.PointCloud) -> None:
        """
        Writes the point cloud to disk for GPD input.
        """
        o3d.io.write_point_cloud(self._gpd_pcd_path, pcd)
        rospy.loginfo(f"Wrote transformed point cloud to {self._gpd_pcd_path}")

    def _run_gpd(self) -> bool:
        """
        Calls the GPD binary on the saved point cloud. Returns True if successful.
        """
        rospy.loginfo("Running GPD grasp detection...")
        try:
            subprocess.run(
                [self._gpd_binary_path, self._gpd_config_path, self._gpd_pcd_path],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
            )
            return True
        except subprocess.CalledProcessError:
            rospy.logwarn("GPD failed. No grasps will be returned.")
            return False

    def _read_and_parse_gpd_output(
        self, frame_id: str
    ) -> Tuple[List[PoseStamped], List[float]]:
        """
        Reads and parses the GPD output file into grasp poses and scores.
        """
        grasps: List[PoseStamped] = []
        scores: List[float] = []

        if not os.path.exists(self._gpd_output_file):
            rospy.logwarn("GPD output file not found. No grasps will be returned.")
            return grasps, scores

        with open(self._gpd_output_file, "r") as fp:
            lines = fp.readlines()

        for line in lines:
            x = np.array(line.strip().split(","), dtype=float)

            position = x[:3]
            axis = x[3:6]
            approach = x[6:9]
            binormal = x[9:12]
            score = x[13]

            rot_mat = np.stack([approach, binormal, axis], axis=1)
            homogeneous_mat = np.eye(4)
            homogeneous_mat[:3, :3] = rot_mat
            quat = tf.quaternion_from_matrix(homogeneous_mat)

            grasp_pose = Pose(position=Point(*position), orientation=Quaternion(*quat))
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = frame_id
            pose_stamped.pose = grasp_pose

            grasps.append(pose_stamped)
            scores.append(score)

        rospy.loginfo(f"Parsed {len(grasps)} grasps from GPD output.")
        return grasps, scores

    def _postprocess_grasps(
        self,
        mesh: o3d.geometry.PointCloud,
        grasps: List[PoseStamped],
        scores: List[float],
        score_threshold: float = -np.inf,
        surface_threshold: float = 0.01,
        max_shift: float = 0.12,
        step_size: float = 0.01,
        angle_threshold_deg: float = 10.0,
        pregrasp_offset_x: float = -0.15,
    ) -> Tuple[List[PoseStamped], List[float]]:
        """
        Applies postprocessing to grasps.
        """
        original_count = len(grasps)
        grasps, scores = self._filter_by_score(grasps, scores, score_threshold)
        grasps, scores = self._filter_or_shift_to_surface(
            mesh, grasps, scores, surface_threshold, max_shift, step_size
        )
        grasps_base_footprint = self._tf_poses(grasps, "base_footprint")
        grasps_base_footprint, scores = self._filter_by_angles(
            grasps_base_footprint, scores, angle_threshold_deg
        )
        grasps_gripper_frame = self._tf_poses(
            grasps_base_footprint, "gripper_grasping_frame"
        )
        grasps_gripper_frame = self._offset_grasps(
            grasps_gripper_frame, pregrasp_offset_x, 0.0, 0.0
        )
        grasps_gripper_frame = self._offset_grasps_for_calibration(
            grasps_gripper_frame,
            CALIBRATION_OFFSET_X,
            CALIBRATION_OFFSET_Y,
            CALIBRATION_OFFSET_Z,
        )
        grasps_base_footprint = self._tf_poses(grasps_gripper_frame, "base_footprint")
        rospy.loginfo(
            f"Postprocessing and filtering of grasps finished. Kept {len(grasps_base_footprint)} / {original_count} grasps."
        )
        return grasps_base_footprint, scores

    def _filter_by_score(
        self,
        grasps: List[PoseStamped],
        scores: List[float],
        score_threshold: float = 0.0,
    ) -> Tuple[List[PoseStamped], List[float]]:

        original_count = len(grasps)

        filtered_grasps, filtered_scores = [], []
        for grasp, score in zip(grasps, scores):
            if score >= score_threshold:
                filtered_grasps.append(grasp)
                filtered_scores.append(score)

        rospy.loginfo(
            f"Filtered grasps below score threshold ({score_threshold}). Kept {len(filtered_grasps)} / {original_count} grasps."
        )

        return filtered_grasps, filtered_scores

    def _filter_or_shift_to_surface(
        self,
        mesh: o3d.geometry.PointCloud,
        grasps: List[PoseStamped],
        scores: List[float],
        surface_threshold: float = 0.01,
        max_shift: float = 0.12,
        step_size: float = 0.001,
    ) -> Tuple[List[PoseStamped], List[float]]:

        original_count = len(grasps)
        filtered_grasps, filtered_scores = [], []

        # Make kd-tree
        kdtree = o3d.geometry.KDTreeFlann(mesh)

        for grasp, score in zip(grasps, scores):
            pos = np.array(
                [grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z]
            )
            rot_mat = o3d.geometry.get_rotation_matrix_from_quaternion(
                [
                    grasp.pose.orientation.w,
                    grasp.pose.orientation.x,
                    grasp.pose.orientation.y,
                    grasp.pose.orientation.z,
                ]
            )
            approach = rot_mat[:, 0]  # Local x-axis
            _, _, dists = kdtree.search_knn_vector_3d(pos, 1)
            nearest_dist = (
                np.sqrt(dists[0]) if len(dists) > 0 else np.inf
            )  # Distance to closest point in the tree

            if nearest_dist <= surface_threshold:
                # Already on surface
                filtered_grasps.append(grasp)
                filtered_scores.append(score)
                continue

            # Shift positively along the local x-axis
            shifted_grasp = None
            for d in np.arange(0, max_shift + step_size, step_size):
                shifted_pos = pos + d * approach
                _, _, dists = kdtree.search_knn_vector_3d(shifted_pos, 1)
                if len(dists) == 0:
                    continue
                dist = np.sqrt(dists[0])  # New closest distance
                if dist <= surface_threshold:
                    new_grasp = deepcopy(grasp)
                    new_grasp.pose.position.x = shifted_pos[0]
                    new_grasp.pose.position.y = shifted_pos[1]
                    new_grasp.pose.position.z = shifted_pos[2]
                    shifted_grasp = new_grasp
                    break
            if shifted_grasp is not None:
                filtered_grasps.append(shifted_grasp)
                filtered_scores.append(score)

        rospy.loginfo(
            f"Filtered grasps above surface threshold ({surface_threshold}), even after shifting. Kept {len(filtered_grasps)} / {original_count} grasps."
        )

        return filtered_grasps, filtered_scores

    def _filter_by_angles(
        self,
        grasps: List[PoseStamped],
        scores: List[float],
        angle_threshold_deg: float = 25.0,
    ) -> Tuple[List[PoseStamped], List[float]]:
        original_count = len(grasps)

        filtered_grasps, filtered_scores = [], []

        dirs = [
            np.array([1, 0, 0]),  # front
            np.array([-1, 0, 0]),  # back
            np.array([0, 1, 0]),  # left
            np.array([0, -1, 0]),  # right
            np.array([0, 0, -1]),  # top-down
            np.array([0, 0, 1]),  # bottom-up
        ]
        threshold_rad = np.deg2rad(angle_threshold_deg)

        def angle_between(v1: np.ndarray, v2: np.ndarray) -> float:
            """
            Returns the angle (in radians) between two vectors.
            """
            v1_u = v1 / np.linalg.norm(v1)
            v2_u = v2 / np.linalg.norm(v2)
            return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

        for grasp, score in zip(grasps, scores):
            rot = o3d.geometry.get_rotation_matrix_from_quaternion(
                [
                    grasp.pose.orientation.x,
                    grasp.pose.orientation.y,
                    grasp.pose.orientation.z,
                    grasp.pose.orientation.w,
                ]
            )
            x_axis = rot[:, 0]  # Local x-axis

            if any(angle_between(x_axis, ref) < threshold_rad for ref in dirs):
                filtered_grasps.append(grasp)
                filtered_scores.append(score)

        rospy.loginfo(
            f"Filtered grasps above angle threshold ({angle_threshold_deg}). Kept {len(filtered_grasps)} / {original_count} grasps."
        )

        return filtered_grasps, filtered_scores

    def _tf_poses(
        self, poses: List[PoseStamped], target_frame: str
    ) -> List[PoseStamped]:
        transformed_poses = []
        if not poses:
            return poses
        source_frame = poses[0].header.frame_id
        try:
            # Wait for the transform to be available
            self._tf_buffer.can_transform(
                target_frame, source_frame, rospy.Time(0), timeout=rospy.Duration(2.0)
            )
            transform = self._tf_buffer.lookup_transform(
                target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0)
            )

            for pose in poses:
                transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
                transformed_poses.append(transformed)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logerr(f"Transform error from {source_frame} to {target_frame}: {e}")
            return []

        return transformed_poses

    def _offset_grasps(
        self,
        grasps: List[PoseStamped],
        dx: float,
        dy: float,
        dz: float,
    ) -> List[PoseStamped]:
        offset_grasps = []
        for grasp in grasps:
            rot = tf.quaternion_matrix(
                [
                    grasp.pose.orientation.x,
                    grasp.pose.orientation.y,
                    grasp.pose.orientation.z,
                    grasp.pose.orientation.w,
                ]
            )
            # Apply offset
            offset_local = np.array([dx, dy, dz, 1.0])
            pos = np.array(
                [
                    grasp.pose.position.x,
                    grasp.pose.position.y,
                    grasp.pose.position.z,
                ]
            )
            offset = rot @ offset_local
            new_pos = pos + offset[:3]
            new_grasp = deepcopy(grasp)
            new_grasp.pose.position.x = new_pos[0]
            new_grasp.pose.position.y = new_pos[1]
            new_grasp.pose.position.z = new_pos[2]
            offset_grasps.append(new_grasp)

        return offset_grasps

    def _offset_grasps_for_calibration(
        self,
        grasps: List[PoseStamped],
        dx: float,
        dy: float,
        dz: float,
    ) -> List[PoseStamped]:
        offset_grasps = []

        for grasp in grasps:
            rot = tf.quaternion_matrix(
                [
                    grasp.pose.orientation.x,
                    grasp.pose.orientation.y,
                    grasp.pose.orientation.z,
                    grasp.pose.orientation.w,
                ]
            )

            rot_inv = np.linalg.inv(rot)
            offset_world = np.array([dx, dy, dz, 0.0])
            offset_local = rot_inv @ offset_world
            offset_vector = rot @ np.array(
                [offset_local[0], offset_local[1], offset_local[2], 1.0]
            )
            pos = np.array(
                [
                    grasp.pose.position.x,
                    grasp.pose.position.y,
                    grasp.pose.position.z,
                ]
            )
            new_pos = pos + offset_vector[:3]
            new_grasp = deepcopy(grasp)
            new_grasp.pose.position.x = new_pos[0]
            new_grasp.pose.position.y = new_pos[1]
            new_grasp.pose.position.z = new_pos[2]
            offset_grasps.append(new_grasp)

        return offset_grasps

    def _get_final_grasp_pose(self) -> PoseStamped:
        # Get current pose of the end-effector
        eef_pose = self._get_eef_pose(base_frame="gripper_grasping_frame")
        x_forward = 0.09  # fixed, for now
        eef_forward_pose = self._offset_grasps([eef_pose], x_forward, 0.0, 0.0)[0]
        eef_forward_pose_base_footprint = self._tf_poses(
            [eef_forward_pose], "base_footprint"
        )[0]
        return eef_forward_pose_base_footprint

    def _get_post_grasp_pose(self) -> PoseStamped:
        # Get current pose of the end-effector
        eef_pose = self._get_eef_pose(base_frame="base_footprint")
        z_up = 0.1
        eef_pose.pose.position.z += z_up
        return eef_pose

    def _get_eef_pose(
        self,
        ee_frame: str = "gripper_grasping_frame",
        base_frame: str = "base_footprint",
    ) -> Pose:
        try:
            trans = self._tf_buffer.lookup_transform(
                base_frame, ee_frame, rospy.Time(0), rospy.Duration(1.0)
            )
            pose = Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            pose.orientation = trans.transform.rotation
            return PoseStamped(pose=pose, header=Header(frame_id=base_frame))
        except Exception as e:
            rospy.logerr(
                f"Failed to get transform from {base_frame} to {ee_frame}: {e}"
            )
            return None

    def _eef_is_fully_closed(self) -> bool:
        controller_state = rospy.wait_for_message(
            "/parallel_gripper_controller/state", JointTrajectoryControllerState
        )
        return controller_state.desired.positions[0] == 0.0


if __name__ == "__main__":
    rospy.init_node("lasr_manipulation_picking")
    pick_server = PickServer()
    rospy.spin()
