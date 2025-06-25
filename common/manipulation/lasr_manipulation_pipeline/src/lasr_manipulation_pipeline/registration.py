#!/usr/bin/env python
import copy
import pathlib
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import time
import open3d as o3d
import numpy as np
import copy
from typing import Tuple
import math


def _quality_ok(fitness: float,
                rmse: float,
                res: float,
                fit_thresh: float = 0.80,
                rmse_factor: float = 1.0) -> bool:
    return (fitness >= fit_thresh) and (rmse <= rmse_factor * res)

def generate_fake_data(pcd):
    """
    Apply a similarity transform (scale + rotation) to a point cloud.
    Used to create synthetic partial data for testing.
    """
    T = np.eye(4)
    s = 1.0
    yaw, pitch, roll = 30, -10, 80
    R_mat = R.from_euler("ZYX", [yaw, pitch, roll], degrees=True).as_matrix()
    T[:3, :3] = s * R_mat
    pcd.transform(T)
    return pcd

def pre_align_pca(pcd: o3d.geometry.PointCloud) -> np.ndarray:
    obb = pcd.get_oriented_bounding_box()
    R_obb = obb.R       
    c = obb.center       
    T = np.eye(4)
    T[:3, :3] = R_obb.T
    T[:3,  3] = -R_obb.T @ c
    pcd.transform(T)     
    return T


def denoise_point_cloud(target):
    # 2. (Optional) Denoise the partial target input
    target, _ = target.remove_statistical_outlier(nb_neighbors=30, std_ratio=2.0)
    diag = np.linalg.norm(target.get_max_bound() - target.get_min_bound())
    target, _ = target.remove_radius_outlier(nb_points=15, radius=diag * 0.05)

    # 3. (Optional) Remove points far from the main cluster
    data = np.asarray(target.points)
    centroid = data.mean(axis=0)
    dists = np.linalg.norm(data - centroid, axis=1)
    threshold = np.median(dists) * 2.0
    good_idx = np.where(dists < threshold)[0]
    target = target.select_by_index(good_idx)
    return target


def estimate_resolution(pcd, sample_ratio=0.1, max_samples=1000):
    """
    Estimate the median nearest-neighbor distance of a point cloud.
    """
    pts = np.asarray(pcd.points)
    n = pts.shape[0]
    # random sampling
    m = min(max_samples, max(int(n * sample_ratio), 100))
    idx = np.random.choice(n, m, replace=False)
    sampled = pts[idx]
    # build KD-tree
    kdt = o3d.geometry.KDTreeFlann(pcd)
    dists = []
    for p in sampled:
        _, _, sqd = kdt.search_knn_vector_3d(p, 2)
        dists.append(np.sqrt(sqd[1]))
    return float(np.median(dists))


def estimate_voxel_size(pcd, target_points=20000):
    """
    Compute a voxel size so that downsampling yields ~target_points points.
    """
    res = estimate_resolution(pcd)
    m = np.asarray(pcd.points).shape[0]
    scale = (m / target_points) ** (1/3)
    return res * scale


def estimate_denoise_params(pcd):
    """
    Adaptively choose parameters for statistical and radius outlier removal.
    """
    res = estimate_resolution(pcd)
    # std_ratio tighter for dense clouds, looser for sparse
    std_ratio = float(np.clip(res / (res + 1e-6) * 2.0, 1.0, 3.0))
    # radius threshold in [2, 10] * res
    radius = float(np.clip(5.0 * res, 2.0 * res, 10.0 * res))
    nb_neighbors = int(np.clip(20 * (res + 1e-6), 10, 50))
    min_points = int(np.clip(10 * (res + 1e-6), 5, 30))
    return {
        'nb_neighbors': nb_neighbors,
        'std_ratio': std_ratio,
        'radius': radius,
        'min_points': min_points
    }


def estimate_ransac_params(res, p_inlier=0.05, sample_size=3,
                            success_prob=0.99, factors=(3, 5, 7)):
    """
    Compute adaptive RANSAC thresholds and iteration count.
    """
    # max correspondence distances
    distances = [res * f for f in factors]
    # number of iterations for desired success probability
    import math
    its = int(math.log(1 - success_prob) / math.log(1 - p_inlier ** sample_size))
    return distances, max(its, 1000)


def estimate_icp_scales(res, max_scale=4.0, min_scale=0.5, steps=4):
    """
    Generate a geometric sequence of ICP scales from max to min.
    """
    return list(np.geomspace(max_scale, min_scale, num=steps))


def preprocess(pcd, voxel, compute_fpfh=True):
    """
    Downsample, estimate normals, and optionally compute FPFH features.
    """
    down = pcd.voxel_down_sample(voxel)
    down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30)
    )
    fpfh = None
    if compute_fpfh:
        fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5, max_nn=100)
        )
    return down, fpfh


def register_object(
    source_pcd: o3d.geometry.PointCloud,
    target_pcd: o3d.geometry.PointCloud,
    target_points: int = 20000,
    downsample: bool = True,
    verbose: bool = False
) -> Tuple[np.ndarray, float, float]:
    """
    Fully automatic RANSAC+ICP registration with self-tuning parameters.
    """
    # 1. Denoise both clouds
    d_params = estimate_denoise_params(source_pcd)
    def denoise(p):
        pc, _ = p.remove_statistical_outlier(
            nb_neighbors=d_params['nb_neighbors'],
            std_ratio=d_params['std_ratio']
        )
        pc, _ = pc.remove_radius_outlier(
            nb_points=d_params['min_points'],
            radius=d_params['radius']
        )
        return pc

    src_clean = denoise(copy.deepcopy(source_pcd))
    tgt_clean = denoise(copy.deepcopy(target_pcd))

    T_pca = pre_align_pca(src_clean)

    # 2. Compute resolution and voxel size
    res = estimate_resolution(src_clean)
    voxel = estimate_voxel_size(src_clean, target_points)
    if verbose:
        print(f"Estimated resolution={res:.4f}, voxel_size={voxel:.4f}")

    # 3. Preprocess (downsample + features)
    if downsample:
        src_down, src_fpfh = preprocess(src_clean, voxel, True)
        tgt_down, tgt_fpfh = preprocess(tgt_clean, voxel, True)
    else:
        src_clean.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30)
        )
        tgt_clean.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30)
        )
        src_down, src_fpfh = src_clean, None
        tgt_down, tgt_fpfh = tgt_clean, None

    # 4. Adaptive RANSAC
    distances, ransac_iters = estimate_ransac_params(res)
    best_ransac = None
    src_down.normalize_normals()
    tgt_down.normalize_normals()
    for dist in distances:
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            src_down, tgt_down, src_fpfh, tgt_fpfh,
            mutual_filter=True,
            max_correspondence_distance=dist,
            estimation_method=o3d.pipelines.registration.
                TransformationEstimationPointToPoint(False),
            ransac_n=3,
            criteria=o3d.pipelines.registration.
                RANSACConvergenceCriteria(ransac_iters, 1000),
            checkers=[
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(dist),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                # o3d.pipelines.registration.CorrespondenceCheckerBasedOnNormal(
                #     math.radians(60.0)  
                # )
            ],
        )
        if verbose:
            print(f"[RANSAC dist={dist:.4f}] fitness={result.fitness:.3f}")
        if best_ransac is None or result.fitness > best_ransac.fitness:
            best_ransac = result

    # 5. Re-estimate normals for full clouds
    for pc in (src_clean, tgt_clean):
        pc.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30)
        )
        pc.orient_normals_consistent_tangent_plane(50)

    # 6. Adaptive multi-scale ICP
    scales = estimate_icp_scales(res)
    current_T = best_ransac.transformation
    best_icp = None
    best_score = -np.inf
    for s in scales:
        thresh = res * s
        icp = o3d.pipelines.registration.registration_icp(
            src_clean, tgt_clean,
            max_correspondence_distance=thresh,
            init=current_T,
            estimation_method=
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(30)
        )
        score = icp.fitness - 0.3 * (icp.inlier_rmse / thresh)
        if score > best_score:
            best_score, best_icp = score, icp
        current_T = icp.transformation
        if verbose:
            print(f"[ICP thresh={thresh:.4f}] fit={icp.fitness:.3f}")
    T = best_icp.transformation.copy()
    R = T[:3, :3]
    if np.linalg.det(R) < 0:
        R[:, 2] *= -1
        T[:3, :3] = R
    T = T @ T_pca
    return T, best_ransac.fitness, best_icp.fitness

def register_object_service(
    src_pcd: o3d.geometry.PointCloud,
    tgt_pcd: o3d.geometry.PointCloud,
    *,
    max_attempts: int = 10,
    fit_thresh: float = 0.30,
    rmse_factor: float = 3.0,
    **kwargs
):

    res = estimate_resolution(src_pcd)            
    best_T, best_fit, best_rmse = None, -1.0, np.inf

    for k in range(1, max_attempts + 1):
        print(f"\n=== Attempt {k}/{max_attempts} ===")

        # --- run your original registration routine ---
        T, fit_ransac, fit_icp = register_object(
            src_pcd, tgt_pcd, **kwargs
        )
        rmse = getattr(fit_icp, "inlier_rmse", 0.0)

        print(f"fitness = {fit_icp:.3f}, rmse = {rmse:.4f}")

        # Keep the best result so far
        if fit_icp > best_fit:
            best_T, best_fit, best_rmse = T, fit_icp, rmse

        # Check quality; exit early if constraints are satisfied
        if _quality_ok(fit_icp, rmse, res, fit_thresh, rmse_factor):
            print(" Quality met — exiting early")
            return best_T, best_fit, best_rmse, k

        # (Optional) Insert tweaks here:
        #   e.g. jitter target, increase RANSAC thresholds, etc.
        # Current version simply retries with the same parameters

    print("[WARN] Quality target not reached after all attempts — "
          "returning best found result")
    return best_T, best_fit, best_rmse, max_attempts
