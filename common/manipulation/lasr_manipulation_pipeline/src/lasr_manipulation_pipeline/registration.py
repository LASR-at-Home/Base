from typing import Tuple
import open3d as o3d
import numpy as np


from typing import Tuple
import numpy as np
import open3d as o3d
import copy

from typing import Tuple
import numpy as np
import open3d as o3d
import copy


def register_object(
    source_pcd: o3d.geometry.PointCloud,
    target_pcd: o3d.geometry.PointCloud,
    voxel_size: float = None,
    downsample: bool = True,
    verbose: bool = False,
) -> Tuple[np.ndarray, float, float]:
    """
    Registers a source point cloud to a target point cloud using RANSAC + ICP.

    Args:
        source_pcd: The source point cloud (e.g., object model).
        target_pcd: The target point cloud (e.g., partial scene).
        voxel_size: Optional voxel size for downsampling. If None, auto-computed.
        downsample: Whether to apply voxel downsampling.
        verbose: If True, prints debug information.

    Returns:
        (4x4 transformation matrix, RANSAC fitness, ICP fitness)
    """

    def auto_voxel(pcd):
        diag = np.linalg.norm(pcd.get_max_bound() - pcd.get_min_bound())
        return diag / 60.0

    def preprocess(pcd, voxel):
        down = pcd.voxel_down_sample(voxel)
        down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30)
        )
        fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5, max_nn=100),
        )
        return down, fpfh

    def denoise(pcd):
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=2.0)
        diag = np.linalg.norm(pcd.get_max_bound() - pcd.get_min_bound())
        pcd, _ = pcd.remove_radius_outlier(nb_points=15, radius=diag * 0.05)
        return pcd

    source = denoise(copy.deepcopy(source_pcd))
    target = denoise(copy.deepcopy(target_pcd))

    if voxel_size is None:
        voxel_size = auto_voxel(source)

    if verbose:
        print(f"Using voxel_size: {voxel_size:.4f}")

    if downsample:
        src_down, src_fpfh = preprocess(source, voxel_size)
        tgt_down, tgt_fpfh = preprocess(target, voxel_size)
    else:
        source.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30)
        )
        target.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30)
        )
        src_down, src_fpfh = source, None
        tgt_down, tgt_fpfh = target, None

    # Multi-distance RANSAC
    thresholds = [4, 6, 8]
    best_ransac = None
    for coef in thresholds:
        max_dist = voxel_size * coef
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            src_down,
            tgt_down,
            src_fpfh,
            tgt_fpfh,
            mutual_filter=True,
            max_correspondence_distance=max_dist,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(
                False
            ),
            ransac_n=3,
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
                500_000, 4000
            ),
            checkers=[
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                    max_dist
                ),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            ],
        )
        if verbose:
            print(
                f"[RANSAC] dist={max_dist:.4f} fitness={result.fitness:.3f} rmse={result.inlier_rmse:.4f}"
            )
        if best_ransac is None or result.fitness > best_ransac.fitness:
            best_ransac = result

    # Refine normals for full ICP
    for pcd in (source, target):
        pcd.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30)
        )
        pcd.orient_normals_consistent_tangent_plane(50)

    # Multi-scale ICP refinement
    current_T = best_ransac.transformation
    best_icp = None
    best_score = -np.inf
    for factor in [2.0, 1.0, 0.5]:
        max_corr = voxel_size * factor
        result_icp = o3d.pipelines.registration.registration_icp(
            source,
            target,
            max_correspondence_distance=max_corr,
            init=current_T,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=30
            ),
        )
        score = result_icp.fitness - 0.3 * (result_icp.inlier_rmse / max_corr)
        if score > best_score:
            best_icp = result_icp
            best_score = score
        current_T = result_icp.transformation

        if verbose:
            print(
                f"[ICP] @ voxel*{factor:.1f} → fit={result_icp.fitness:.3f} rmse={result_icp.inlier_rmse:.4f}"
            )

    return best_icp.transformation, best_ransac.fitness, best_icp.fitness
