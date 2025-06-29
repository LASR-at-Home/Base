from typing import List

import open3d as o3d
import numpy as np
from lasr_manipulation_pipeline.conversions import pose_to_o3d_frame

from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2


def visualize_grasps_on_scene(
    scene_pcd: o3d.geometry.PointCloud, grasps: List[Pose], window_name: str
) -> None:
    """
    Visualize scene point cloud with grasp coordinate frames.

    Args:
        scene_pcd: open3d.geometry.PointCloud in camera frame`
        grasps: list of ROS Pose objects representing grasp poses
    """
    # Paint the scene point cloud for contrast
    scene_pcd.paint_uniform_color([0.0, 1.0, 0.0])  # green

    # Create Open3D frames for each grasp
    grasp_frames = [pose_to_o3d_frame(g, size=0.03) for g in grasps]

    # Show everything together
    o3d.visualization.draw_geometries(
        [scene_pcd, *grasp_frames], window_name=window_name
    )
