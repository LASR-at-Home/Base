from typing import Tuple, List, Union, Optional

import open3d as o3d
import tf.transformations
import subprocess
import numpy as np

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
        if score > 0.0:
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


def filter_grasps_on_surface(
    pcd: o3d.geometry.PointCloud,
    poses: List[Pose],
    approaches: List[Vector3],
    scores: List[float],
    openings: List[float],
    surface_threshold: float = 0.01,
) -> Tuple[List[Pose], List[Vector3], List[float], List[float]]:
    """
    Filter grasps to only include those within a distance threshold of the object's surface.
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
        [_, idx, dists] = kdtree.search_knn_vector_3d(grasp_pos, 1)
        nearest_dist = np.sqrt(dists[0]) if dists else np.inf

        if nearest_dist <= surface_threshold:
            filtered_poses.append(pose)
            filtered_approaches.append(approach)
            filtered_scores.append(score)
            filtered_openings.append(opening)

    filtered_count = len(filtered_scores)
    print(
        f"[Surface Filter] Kept {filtered_count} / {original_count} grasps (threshold: {surface_threshold:.4f} m)"
    )

    return filtered_poses, filtered_approaches, filtered_scores, filtered_openings
