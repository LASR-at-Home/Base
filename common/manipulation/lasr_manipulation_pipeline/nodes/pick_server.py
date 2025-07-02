from typing import List, Tuple

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

import tf2_geometry_msgs
import tf2_ros

from lasr_manipulation_msgs.msg import PickAction, PickGoal, PickResult


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
        rospkg.RosPack().get_path("lasr_manipulation_pipeline"), "cfg", "tiago.cfg"
    )
    _gpd_pcd_path: str = "/tmp/gpd.pcd"
    _gpd_output_file: str = "grasps.txt"

    # MoveIt
    _move_group: MoveGroupCommander
    _close_gripper: rospy.ServiceProxy

    # Tf
    _tf_buffer: tf2_ros.Buffer
    _tf_listener: tf2_ros.TransformListener

    # Action server
    _pick_server: actionlib.SimpleActionServer

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

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._pick_server.start()

        rospy.loginfo("/lasr_manipulation/pick has started!")

    def _pick(self, goal: PickGoal) -> None:
        object_id: str = goal.object_id
        mesh_name: str = goal.mesh_name
        transform: TransformStamped = goal.transform
        scale: Vector3Stamped = goal.scale

        # Generate grasps
        grasps, scores = self._generate_grasps(mesh_name, transform, scale)
        # If no grasps are outputted, finish.
        if not grasps:
            result = PickResult(success=False)
            self._pick_server.set_aborted(result)
            return

        # Clear any existing pose targets
        self._move_group.clear_pose_targets()
        # Execute pre-grasos
        rospy.loginfo("Dispatching pre-grasps to MoveIt for execution...")
        self._move_group.set_pose_reference_frame("gripper_grasping_frame")
        self._move_group.set_pose_targets(grasps)
        success = self._move_group.go(wait=True)
        if not success:
            rospy.logwarn("MoveIt failed to execute pre-grasps. Aborting.")
            result = PickResult(sucess=False)
            self._pick_server.set_aborted(result)
            return

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
    ) -> o3d.geometry.PointCloud | None:
        """
        Loads the mesh, applies a transformation and scaling, and returns the processed point cloud.
        """
        mesh_path = os.path.join(self._mesh_dir, f"{mesh_name}.ply")
        mesh = o3d.io.read_triangle_mesh(mesh_path)

        if mesh.is_empty():
            rospy.logwarn("Mesh is empty, so no grasps will be generated.")
            return None

        pcd = mesh.sample_points_poisson_disk(number_of_points=10000, init_factor=5)
        rospy.loginfo(f"Read a mesh with {pcd.points.shape[0]} points.")

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
        score_threshold: float = 0.0,
        surface_threshold: float = 0.01,
        max_shift: float = 0.12,
        step_size: float = 0.001,
        angle_threshold_deg: float = 25.0,
        pregrasp_offset_x: float = 0.12,
    ) -> Tuple[List[PoseStamped], List[float]]:
        """
        Applies postprocessing to grasps.
        """
        original_count = len(grasps)
        grasps, scores = self._filter_by_score(grasps, scores, score_threshold)
        grasps, scores = self._filter_or_shift_to_surface(
            mesh, grasps, scores, surface_threshold, max_shift, step_size
        )
        grasps, scores = self._filter_by_angles(grasps, scores, angle_threshold_deg)
        grasps = self._offset_grasps(grasps, pregrasp_offset_x, 0.0, 0.0)
        rospy.loginfo(
            f"Postprocessing and filtering of grasps finished. Kept {len(grasps)} / {original_count} grasps."
        )
        return grasps, scores

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
                    grasp.pose.orientation.x,
                    grasp.pose.orientation.y,
                    grasp.pose.orientation.z,
                    grasp.pose.orientation.w,
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

    def _offset_grasps(
        self,
        grasps: List[PoseStamped],
        dx: float,
        dy: float,
        dz: float,
        local_frame: str = "gripper_grasping_frame",
    ) -> List[PoseStamped]:
        offset_grasps = []
        source_frame = grasps[0].header.frame_id
        try:
            # Get transform to local frame
            self._tf_buffer.can_transform(
                local_frame,
                source_frame,
                rospy.Time(0),
                timeout=rospy.Duration(2.0),
            )
            transform = self._tf_buffer.lookup_transform(
                local_frame, source_frame, rospy.Time(0), rospy.Duration(1.0)
            )
            for grasp in grasps:
                # Transform to local frame
                grasp_local_frame = tf2_geometry_msgs.do_transform_pose(
                    grasp, transform
                )
                rot = tf.quaternion_matrix(
                    [
                        grasp_local_frame.pose.orientation.x,
                        grasp_local_frame.pose.orientation.y,
                        grasp_local_frame.pose.orientation.z,
                        grasp_local_frame.pose.orientation.w,
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
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logwarn(
                f"Transform error from {source_frame} to {local_frame}. No grasps will be returned. {e}"
            )
            return []


if __name__ == "__main__":
    rospy.init_node("/lasr_manipulation")
    pick_server = PickServer()
    rospy.spin()
