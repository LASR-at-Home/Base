from typing import Tuple, List, Optional

import open3d as o3d
import tf.transformations
import subprocess
import numpy as np
import copy

from geometry_msgs.msg import Pose, Point, Quaternion, Vector3


def grasp_to_pose(
    position: np.ndarray,
    axis: np.ndarray,
    approach: np.ndarray,
    binormal: np.ndarray,
) -> Pose:
    rotation_matrix = np.stack([approach, binormal, axis], axis=1)
    homogeneous_matrix = np.eye(4)
    homogeneous_matrix[:3, :3] = rotation_matrix
    quaternion = tf.transformations.quaternion_from_matrix(homogeneous_matrix)
    return Pose(
        position=Point(*position), orientation=Quaternion(*quaternion)
    ), Vector3(*approach)


def grasp_from_str(grasp_str: str) -> Tuple[Pose, Vector3, float, float]:
    xs = np.array(grasp_str.split(","), dtype=float)
    position = xs[:3]
    axis = xs[3:6]
    approach = xs[6:9]
    binormal = xs[9:12]
    opening = xs[12]  # unused
    score = xs[13]
    pose, approach = grasp_to_pose(position, axis, approach, binormal)
    return pose, approach, score, opening


def generate_grasps(
    path_to_pcd: str,
    path_to_gpd: str,
    path_to_gpd_cfg: str,
    pcd: Optional[o3d.geometry.PointCloud] = None,
) -> Tuple[List[Pose], List[Vector3], List[float], List[float]]:

    if pcd is not None:
        o3d.io.write_point_cloud(path_to_pcd, pcd)

    # Run GPD executable
    subprocess.run([path_to_gpd, path_to_gpd_cfg, path_to_pcd])

    # Parse grasps
    with open("grasps.txt", "r") as fp:
        output = fp.readlines()

    # Build lists
    poses = []
    approaches = []
    scores = []
    openings = []
    for line in output:
        pose, approach, score, opening = grasp_from_str(line)
        poses.append(pose)
        approaches.append(approach)
        scores.append(score)
        openings.append(opening)

    sorted_pairs = sorted(
        zip(poses, approaches, scores, openings), key=lambda x: x[2], reverse=True
    )
    poses, approaches, scores, openings = zip(*sorted_pairs)
    poses = list(poses)
    approaches = list(approaches)
    scores = list(scores)
    openings = list(openings)

    return poses, approaches, scores, openings


def filter_grasps_by_score(
    poses: List[Pose],
    approaches: List[Vector3],
    scores: List[float],
    openings: List[float],
    score_threshold: float = 0.0,
) -> Tuple[List[Pose], List[Vector3], List[float], List[float]]:
    """
    Filter grasps below a given score.
    """
    original_count = len(scores)

    filtered_poses = []
    filtered_approaches = []
    filtered_scores = []
    filtered_openings = []

    for pose, approach, score, opening in zip(poses, approaches, scores, openings):
        if score >= score_threshold:
            filtered_poses.append(pose)
            filtered_approaches.append(approach)
            filtered_scores.append(score)
            filtered_openings.append(opening)

    filtered_count = len(filtered_scores)
    print(
        f"[Score Filter] Kept {filtered_count} / {original_count} grasps (threshold: {score_threshold})"
    )

    return filtered_poses, filtered_approaches, filtered_scores, filtered_openings


def shift_or_filter_grasps_to_surface(
    pcd: o3d.geometry.PointCloud,
    poses: List[Pose],
    approaches: List[Vector3],
    scores: List[float],
    openings: List[float],
    surface_threshold: float = 0.01,
    max_shift: float = 0.12,
    step_size: float = 0.001,
) -> Tuple[List[Pose], List[Vector3], List[float], List[float]]:
    """
    Filter grasps within a distance threshold of the surface,
    shifting them forward along the local X-axis (approach direction)
    if not close enough.
    """
    if len(pcd.points) == 0:
        raise ValueError("Point cloud is empty.")

    original_count = len(poses)
    kdtree = o3d.geometry.KDTreeFlann(pcd)

    filtered_poses = []
    filtered_approaches = []
    filtered_scores = []
    filtered_openings = []

    for pose, approach, score, opening in zip(poses, approaches, scores, openings):
        grasp_pos = np.array([pose.position.x, pose.position.y, pose.position.z])

        # Rotation matrix from quaternion (x,y,z,w)
        qx, qy, qz, qw = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        R = o3d.geometry.get_rotation_matrix_from_quaternion([qw, qx, qy, qz])
        approach_dir = R[:, 0]  # Local X axis (approach direction)

        # Distance at original position
        _, idx, dists = kdtree.search_knn_vector_3d(grasp_pos, 1)
        nearest_dist = np.sqrt(dists[0]) if len(dists) > 0 else np.inf

        if nearest_dist <= surface_threshold:
            filtered_poses.append(pose)
            filtered_approaches.append(approach)
            filtered_scores.append(score)
            filtered_openings.append(opening)
            continue

        # Try shifting forward along +X axis (approach direction)
        shifted_pose = None
        for d in np.arange(0, max_shift + step_size, step_size):
            shifted_pos = grasp_pos + d * approach_dir  # SHIFT FORWARD now
            _, idx, dists = kdtree.search_knn_vector_3d(shifted_pos, 1)
            if len(dists) == 0:
                continue
            dist = np.sqrt(dists[0])
            if dist <= surface_threshold:
                new_pose = copy.deepcopy(pose)
                new_pose.position.x = shifted_pos[0]
                new_pose.position.y = shifted_pos[1]
                new_pose.position.z = shifted_pos[2]
                shifted_pose = new_pose
                break

        if shifted_pose is not None:
            filtered_poses.append(shifted_pose)
            filtered_approaches.append(approach)
            filtered_scores.append(score)
            filtered_openings.append(opening)

    filtered_count = len(filtered_poses)
    print(
        f"[Surface Filter + Shift] Kept {filtered_count} / {original_count} grasps (threshold: {surface_threshold:.4f} m)"
    )

    return filtered_poses, filtered_approaches, filtered_scores, filtered_openings


def angle_between(v1: np.ndarray, v2: np.ndarray) -> float:
    """
    Returns the angle (in radians) between two vectors.
    """
    v1_u = v1 / np.linalg.norm(v1)
    v2_u = v2 / np.linalg.norm(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def filter_by_approach_angles(
    grasps: List[Pose],
    angle_threshold_deg: float = 25.0,
) -> List[Pose]:
    """
    Filters grasps so that their x-axis (approach vector) is close to canonical directions
    """

    # Canonical approach directions (unit vectors)
    canonical_dirs = [
        np.array([1, 0, 0]),  # front
        np.array([-1, 0, 0]),  # back
        np.array([0, 1, 0]),  # left
        np.array([0, -1, 0]),  # right
        np.array([0, 0, -1]),  # top-down
        np.array([0, 0, 1]),  # bottom-up
    ]

    threshold_rad = np.deg2rad(angle_threshold_deg)
    filtered_grasps = []

    for pose in grasps:
        quat = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        rot = o3d.geometry.get_rotation_matrix_from_quaternion(quat)
        x_axis = rot[:, 0]  # gripper x-axis (approach)

        if any(angle_between(x_axis, ref) < threshold_rad for ref in canonical_dirs):
            filtered_grasps.append(pose)

    print(f"[Angle Filter] Kept {len(filtered_grasps)} / {len(grasps)} grasps")

    return filtered_grasps
