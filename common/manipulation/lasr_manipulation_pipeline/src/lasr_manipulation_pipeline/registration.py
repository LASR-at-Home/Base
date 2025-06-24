from typing import Tuple
import open3d as o3d
import numpy as np


def register_object(
    source_pcd: o3d.geometry.PointCloud,
    target_pcd: o3d.geometry.PointCloud,
    downsample: bool = False,
    voxel_size: float = 0.01,
    normal_radius_factor: float = 2.0,
    fpfh_radius_factor: float = 5.0,
    ransac_distance_multiplier: float = 1.5,
    icp_distance_multiplier: float = 0.4,
    edge_length_threshold: float = 0.9,
    ransac_n: int = 4,
    ransac_max_iter: int = 1_000_000,
    ransac_max_validations: int = 1000,
) -> Tuple[np.ndarray, float, float]:
    """
    Registers a source point cloud to a target point cloud using FPFH-based global registration
    followed by local refinement using point-to-plane ICP.

    Parameters:
        source_pcd (o3d.geometry.PointCloud): The source point cloud.
        target_pcd (o3d.geometry.PointCloud): The target point cloud.
        downsample (bool): Whether to downsample the point clouds.
        voxel_size (float): Voxel size for downsampling and feature computation.
        normal_radius_factor (float): Multiplier for voxel_size to compute normals.
        fpfh_radius_factor (float): Multiplier for voxel_size to compute FPFH features.
        ransac_distance_multiplier (float): Max correspondence distance multiplier for RANSAC.
        icp_distance_multiplier (float): Max correspondence distance multiplier for ICP.
        edge_length_threshold (float): Threshold for edge length checker.
        ransac_n (int): Number of RANSAC correspondences.
        ransac_max_iter (int): Maximum RANSAC iterations.
        ransac_max_validations (int): Maximum RANSAC validations.

    Returns:
        Tuple[np.ndarray, float]: A tuple of the 4x4 transformation matrix and ICP fitness score.
    """

    if downsample:
        source_pcd = source_pcd.voxel_down_sample(voxel_size)
        target_pcd = target_pcd.voxel_down_sample(voxel_size)

    # Estimate normals
    source_pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_size * normal_radius_factor, max_nn=30
        )
    )
    target_pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_size * normal_radius_factor, max_nn=30
        )
    )

    # Compute FPFH features
    source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        source_pcd,
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_size * fpfh_radius_factor, max_nn=100
        ),
    )
    target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        target_pcd,
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_size * fpfh_radius_factor, max_nn=100
        ),
    )

    # Global registration using RANSAC
    global_registration = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_pcd,
        target_pcd,
        source_fpfh,
        target_fpfh,
        mutual_filter=True,
        max_correspondence_distance=voxel_size * ransac_distance_multiplier,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(
            False
        ),
        ransac_n=ransac_n,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                edge_length_threshold
            ),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                voxel_size * ransac_distance_multiplier
            ),
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
            ransac_max_iter, ransac_max_validations
        ),
    )

    # Local ICP refinement
    refined_registration = o3d.pipelines.registration.registration_icp(
        source_pcd,
        target_pcd,
        max_correspondence_distance=voxel_size * icp_distance_multiplier,
        init=global_registration.transformation,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    )

    return (
        refined_registration.transformation,
        global_registration.fitness,
        refined_registration.fitness,
    )
