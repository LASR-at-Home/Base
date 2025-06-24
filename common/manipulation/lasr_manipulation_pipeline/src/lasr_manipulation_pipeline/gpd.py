from typing import Tuple, List

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
    pcd: o3d.geometry.PointCloud,
    path_to_pcd: str,
    path_to_gpd: str,
    path_to_gpd_cfg: str,
) -> Tuple[List[Pose], List[Vector3], List[float], List[float]]:

    # Write pcd to disk
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
