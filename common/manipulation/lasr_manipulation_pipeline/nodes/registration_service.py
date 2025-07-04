from typing import Optional, Tuple

import rospy
import rospkg
import os
import open3d as o3d
import numpy as np
import tf.transformations as tf
import copy
from concurrent.futures import ThreadPoolExecutor, as_completed


from lasr_manipulation_msgs.srv import (
    Registration,
    RegistrationRequest,
    RegistrationResponse,
)

from geometry_msgs.msg import Vector3Stamped, TransformStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as point_cloud2


def _quality_ok(
    fitness: float,
    rmse: float,
    res: float,
    fit_thresh: float = 0.80,
    rmse_factor: float = 1.0,
) -> bool:
    return (fitness >= fit_thresh) and (rmse <= rmse_factor * res)


def _one_trial(seed, shared):
    np.random.seed(seed)
    (
        src_clean,
        tgt_clean,
        src_down,
        tgt_down,
        src_fpfh,
        tgt_fpfh,
        res,
        T_pca,
        voxel,
    ) = shared
    return pcl_register(
        copy.deepcopy(src_clean),
        copy.deepcopy(tgt_clean),
        src_fpfh,
        tgt_fpfh,
        copy.deepcopy(src_down),
        copy.deepcopy(tgt_down),
        res,
        T_pca,
        voxel,
        verbose=False,
    )


def pre_align_pca(pcd: o3d.geometry.PointCloud) -> np.ndarray:
    obb = pcd.get_oriented_bounding_box()
    R_obb = obb.R
    c = obb.center
    T = np.eye(4)
    T[:3, :3] = R_obb.T
    T[:3, 3] = -R_obb.T @ c
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
    scale = (m / target_points) ** (1 / 3)
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
        "nb_neighbors": nb_neighbors,
        "std_ratio": std_ratio,
        "radius": radius,
        "min_points": min_points,
    }


def estimate_ransac_params(
    res, p_inlier=0.05, sample_size=3, success_prob=0.99, factors=(3, 5, 7)
):
    """
    Compute adaptive RANSAC thresholds and iteration count.
    """
    # max correspondence distances
    distances = [res * f for f in factors]
    # number of iterations for desired success probability
    import math

    its = int(math.log(1 - success_prob) / math.log(1 - p_inlier**sample_size))
    return distances, max(its, 1000)


def estimate_icp_scales(res, max_scale=4.0, min_scale=0.5, steps=4):
    """
    Generate a geometric sequence of ICP scales from max to min.
    """
    return list(np.geomspace(max_scale, min_scale, num=steps))


def preprocess(
    source_pcd: o3d.geometry.PointCloud,
    target_pcd: o3d.geometry.PointCloud,
    target_points: int = 20000,
    downsample: bool = True,
    verbose: bool = False,
):
    # 1. Denoise both clouds
    d_params = estimate_denoise_params(source_pcd)

    def denoise(p):
        pc, _ = p.remove_statistical_outlier(
            nb_neighbors=d_params["nb_neighbors"], std_ratio=d_params["std_ratio"]
        )
        pc, _ = pc.remove_radius_outlier(
            nb_points=d_params["min_points"], radius=d_params["radius"]
        )
        return pc

    def preprocess_function(pcd, voxel, compute_fpfh=True):
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
                down, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5, max_nn=100)
            )
        return down, fpfh

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
        src_down, src_fpfh = preprocess_function(src_clean, voxel, True)
        tgt_down, tgt_fpfh = preprocess_function(tgt_clean, voxel, True)
    else:
        src_clean.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30)
        )
        tgt_clean.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30)
        )
        src_down, src_fpfh = src_clean, None
        tgt_down, tgt_fpfh = tgt_clean, None

    return (
        src_clean,
        tgt_clean,
        src_down,
        tgt_down,
        src_fpfh,
        tgt_fpfh,
        T_pca,
        res,
        voxel,
    )


def pcl_register(
    src_clean: o3d.geometry.PointCloud,
    tgt_clean: o3d.geometry.PointCloud,
    src_fpfh: o3d.pipelines.registration.Feature,
    tgt_fpfh: o3d.pipelines.registration.Feature,
    src_down: o3d.geometry.PointCloud,
    tgt_down: o3d.geometry.PointCloud,
    res: float,
    T_pca: np.ndarray,
    voxel: float,
    verbose: bool = False,
) -> Tuple[np.ndarray, float, float]:
    """
    Fully automatic RANSAC+ICP registration with self-tuning parameters.
    """

    # 4. Adaptive RANSAC
    distances, ransac_iters = estimate_ransac_params(res)
    best_ransac = None
    src_down.normalize_normals()
    tgt_down.normalize_normals()
    for dist in distances:
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            src_down,
            tgt_down,
            src_fpfh,
            tgt_fpfh,
            mutual_filter=False,
            max_correspondence_distance=dist,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(
                False
            ),
            ransac_n=3,
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
                ransac_iters, 1000
            ),
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
            src_clean,
            tgt_clean,
            max_correspondence_distance=thresh,
            init=current_T,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(30),
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


def register_object(
    src_pcd: o3d.geometry.PointCloud,
    tgt_pcd: o3d.geometry.PointCloud,
    *,
    max_attempts: int = 25,
    fit_thresh: float = 0.7,
    rmse_factor: float = 1.0,
    scale: bool = False,
    **kwargs,
):
    res = estimate_resolution(src_pcd)
    best_T, best_fit, best_rmse = None, -1.0, np.inf
    seeds = np.random.randint(0, 2**32 - 1, size=max_attempts)

    if scale:
        # Optional: Estimate scale between source and target and normalize
        scale1 = np.linalg.norm(
            src_pcd.get_max_bound() - src_pcd.get_min_bound()
        ) / np.linalg.norm(tgt_pcd.get_max_bound() - tgt_pcd.get_min_bound())
        scale2 = (bbox_volume(src_pcd) / bbox_volume(tgt_pcd)) ** (1 / 3)
        scale3 = estimate_scale_from_nn(src_pcd, tgt_pcd, k=1, top_k=100)
        scale_ratio = np.mean([scale1, scale2, scale3])

        if abs(scale_ratio - 1.0) > 0.05:
            print(f"Scaling target by ratio {scale_ratio:.4f}")
            tgt_pcd.scale(scale_ratio, center=tgt_pcd.get_center())
        else:
            print("Skipping scaling; size already aligned.")
    else:
        scale_ratio = 1.0
    (
        src_clean,
        tgt_clean,
        src_down,
        tgt_down,
        src_fpfh,
        tgt_fpfh,
        T_pca,
        res,
        voxel,
    ) = preprocess(src_pcd, tgt_pcd, verbose=True)

    shared = (
        src_clean,
        tgt_clean,
        src_down,
        tgt_down,
        src_fpfh,
        tgt_fpfh,
        res,
        T_pca,
        voxel,
    )

    with ThreadPoolExecutor(max_workers=os.cpu_count()) as pool:
        futures = [pool.submit(_one_trial, s, shared) for s in seeds]
        for fut in as_completed(futures):
            T, _, fit_icp = fut.result()
            rmse = getattr(fit_icp, "inlier_rmse", 0.0)

            if fit_icp > best_fit:
                best_T, best_fit, best_rmse = T, fit_icp, rmse
            if _quality_ok(fit_icp, rmse, res, fit_thresh, rmse_factor):
                for f in futures:
                    f.cancel()
                break
    return best_T, scale_ratio, best_fit, best_rmse


def bbox_volume(pcd):
    bounds = pcd.get_max_bound() - pcd.get_min_bound()
    return np.prod(bounds)


def estimate_scale_from_nn(src, tgt, k=1, top_k=100):
    src_pts = np.asarray(src.points)
    tgt_kdtree = o3d.geometry.KDTreeFlann(tgt)
    ratios = []
    for p in src_pts:
        _, idx, _ = tgt_kdtree.search_knn_vector_3d(p, k)
        nn_pts = np.asarray(tgt.points)[idx]
        dists = np.linalg.norm(nn_pts - p, axis=1)
        if len(dists) > 0:
            ratios.append(np.min(dists))
    ratios = sorted(ratios)[:top_k]
    return np.mean(ratios)


class RegistrationService:
    """
    A service for registering meshes with pointclouds.
    """

    # Meshes
    _mesh_dir: str = os.path.join(
        rospkg.RosPack().get_path("lasr_manipulation_pipeline"), "meshes"
    )

    # Service
    _registration_service: rospy.Service

    def __init__(self):

        self._registration_service = rospy.Service(
            "/lasr_manipulation/registration", Registration, self._perform_registration
        )
        rospy.loginfo("/lasr_manipulation/registration service is ready!")

    def _ros_to_o3d(self, pcl_msg: PointCloud2) -> o3d.geometry.PointCloud:
        points = list(
            point_cloud2.read_points(
                pcl_msg, field_names=("x", "y", "z"), skip_nans=True
            )
        )
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        return pcd

    def _transform_to_ros(self, T: np.ndarray, frame_id) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = frame_id

        translation = T[:3, 3]
        rotation = tf.quaternion_from_matrix(T)

        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]
        return transform

    def _scale_to_ros(self, s: float, frame_id: str) -> Vector3Stamped:
        vec = Vector3Stamped()
        vec.header.stamp = rospy.Time.now()
        vec.header.frame_id = frame_id
        vec.vector.x = s
        vec.vector.y = s
        vec.vector.z = s
        return vec

    def _load_mesh(self, mesh_name: str) -> Optional[o3d.geometry.PointCloud]:
        mesh_path = os.path.join(self._mesh_dir, f"{mesh_name}.ply")
        mesh = o3d.io.read_triangle_mesh(mesh_path)

        if mesh.is_empty():
            rospy.logwarn("Mesh is empty, so registration will not be performed.")
            return None

        pcd = mesh.sample_points_poisson_disk(number_of_points=10000, init_factor=5)
        rospy.loginfo(f"Read a mesh with {np.asarray(pcd.points).shape[0]} points.")
        return pcd

    def _perform_registration(
        self, request: RegistrationRequest
    ) -> RegistrationResponse:
        response = RegistrationResponse()

        source = self._load_mesh(request.mesh_name)
        if source is None:
            response.success = False
            return response

        target = self._ros_to_o3d(request.segmented_pcl)

        try:
            best_T, scale_factor, best_fit, best_rmse = register_object(
                source, target, scale=request.scale
            )
            response.transform = self._transform_to_ros(
                best_T, request.segmented_pcl.header.frame_id
            )
            response.scale = self._scale_to_ros(
                scale_factor, request.segmented_pcl.header.frame_id
            )
            response.success = True
        except Exception as e:
            rospy.logerr(f"Registration failed: {e}")
            response.success = False

        return response


if __name__ == "__main__":
    rospy.init_node("lasr_manipulation_registration")
    registration_service = RegistrationService()
    rospy.spin()
