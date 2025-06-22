#!/usr/bin/env python
import copy
import pathlib
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import time
def fake_partial(cloud: o3d.geometry.PointCloud,
                       view_dir=(0, 0, 1),
                       half_aperture_deg=90,
                       keep_ratio=1.0):
    """
    Return the subset of points that lie within `half_aperture_deg`
    degrees of the `view_dir` (measured at the object centre).

    Parameters
    ----------
    cloud : open3d.geometry.PointCloud   # input full cloud
    view_dir : 3-tuple or ndarray        # direction you "look" from
    half_aperture_deg : float            # 90 keeps a half-sphere,
                                         # 60 keeps a circular cone ±60°
    keep_ratio : 0–1                     # optional random thinning

    Returns
    -------
    partial : open3d.geometry.PointCloud
    """
    # make safe copy (works for CUDA / CPU clouds alike)
    if hasattr(cloud, 'to_legacy'):      # tensor → legacy
        cloud_legacy = cloud.to_legacy()
    else:
        cloud_legacy = cloud
    pts = np.asarray(cloud_legacy.points)

    centre = pts.mean(axis=0)
    v = pts - centre                      # vectors from centre to points
    v_norm = np.linalg.norm(v, axis=1, keepdims=True)
    v_norm[v_norm == 0] = 1e-9            # avoid zero division
    v_unit = v / v_norm

    view_dir = np.asarray(view_dir, dtype=float)
    if np.linalg.norm(view_dir) == 0:
        raise ValueError("view_dir must be non-zero")
    view_dir /= np.linalg.norm(view_dir)

    # cosine of angle between each point vector and view_dir
    cosang = v_unit @ view_dir
    cos_thresh = np.cos(np.deg2rad(half_aperture_deg))
    mask = cosang >= cos_thresh           # inside the cone

    if keep_ratio < 1.0:
        rnd = np.random.rand(mask.size)
        mask &= rnd < keep_ratio

    partial = cloud_legacy.select_by_index(np.where(mask)[0])
    return partial

def auto_voxel(pcd, target_pts=500):
    """Pick a voxel so down-sample keeps roughly `target_pts` points."""
    diag = np.linalg.norm(pcd.get_max_bound() - pcd.get_min_bound())
    return diag / 60.0             # empirical: 1/60 of the bounding box

def auto_ransac_iters(n_down):
    """Log-scale RANSAC iterations to the cloud size."""
    from math import log
    return min(500_000, int(5_000 * log(max(n_down, 10))))

def auto_sor_neighbors(n_raw):
    """Statistical outlier: neighbour count grows slowly with point count."""
    from math import log10
    return max(20, int(0.3 * log10(n_raw) * 100))


# ===================== 2. Utility functions =====================
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


def load_and_scale_to_meter(p):
    """
    Read a point cloud (.xyz or .ply).  
    If coordinates are millimeters, convert manually before calling this.
    """
    ext = pathlib.Path(p).suffix.lower()
    fmt = "xyz" if ext == ".xyz" else "ply"
    cloud = o3d.io.read_point_cloud(p, format=fmt)
    return cloud

def bbox_volume(pcd):
    bounds = pcd.get_max_bound() - pcd.get_min_bound()
    return np.prod(bounds)




def preprocess(pcd, voxel):
    """
    Voxel down-sample, estimate normals, compute FPFH descriptors.
    """
    down = pcd.voxel_down_sample(voxel)
    down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30))
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5, max_nn=100))
    return down, fpfh


def coarse_align_by_center(src, tgt):
    """Translate src to the centroid of tgt."""
    src.translate(tgt.get_center() - src.get_center())
    return src

def estimate_cloud_scale(cloud):
    pts = np.asarray(cloud.points)
    center = pts.mean(axis=0)
    return np.sqrt(np.mean(np.sum((pts - center) ** 2, axis=1)))


def get_pca_axes(pcd):
    """Return principal directions (sorted by descending variance)."""
    pts = np.asarray(pcd.points) - np.mean(pcd.points, axis=0)
    cov = np.cov(pts.T)
    eig_vals, eig_vecs = np.linalg.eigh(cov)
    order = np.argsort(eig_vals)[::-1]
    return eig_vecs[:, order]


def align_long_axes(src_pcd, tgt_pcd):
    """Rotate src so its principal axes align with tgt."""
    R_mat = get_pca_axes(tgt_pcd) @ get_pca_axes(src_pcd).T
    src_pcd.rotate(R_mat, center=src_pcd.get_center())
    return R_mat

def get_longest_axis(pcd):
    pts = np.asarray(pcd.points)
    cov = np.cov((pts - pts.mean(axis=0)).T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    return np.sqrt(max(eigvals))  


def print_bounds(pcd, name="cloud"):
    min_bound = np.asarray(pcd.get_min_bound())
    max_bound = np.asarray(pcd.get_max_bound())
    extent = max_bound - min_bound
    print(f"[{name}]")
    print(f"  min  : {min_bound}")
    print(f"  max  : {max_bound}")
    print(f"  size : {extent}\n")


def print_centroid_gap(src, tgt, note=""):
    gap = np.linalg.norm(src.get_center() - tgt.get_center())
    print(f"{note} centroid distance = {gap:.4f} m")


def draw(*geoms, title="Open3D"):
    o3d.visualization.draw_geometries(
        list(geoms), window_name=title, width=960, height=720)
    
def estimate_scale_from_nn(src, tgt, k=1, top_k=100):
    src_pts = np.asarray(src.points)
    tgt_pts = np.asarray(tgt.points)

    src_kd = o3d.geometry.KDTreeFlann(src)
    ratios = []

    for pt in tgt_pts:
        [_, idx, _] = src_kd.search_knn_vector_3d(pt, k)
        if idx:
            dist_src = np.linalg.norm(src_pts[idx[0]])
            dist_tgt = np.linalg.norm(pt)
            if dist_src > 1e-5:
                ratios.append(dist_tgt / dist_src)

    ratios = np.array(ratios)
    ratios = np.sort(ratios)
    return np.median(ratios[:top_k])



# ===================== 3. Main pipeline =====================
def pcl_registraion(full_path, part_path, scale=False, debug=False):
    # 3-1. Load point clouds
    full = load_and_scale_to_meter(full_path)
    if part_path is None:
        partial = fake_partial(full)

    else:
        partial = load_and_scale_to_meter(part_path)
    partial = generate_fake_data(partial)
    # Remove noise
    full_raw_n =  len(full.points)
    full, _ = full.remove_statistical_outlier(nb_neighbors=auto_sor_neighbors(full_raw_n), std_ratio=2.0)
    diag = np.linalg.norm(partial.get_max_bound() - partial.get_min_bound())
    partial, _ = partial.remove_radius_outlier(nb_points=15, radius=diag * 0.05)

    if scale: 
        # no need to use rescale cause depth camera use real size
        # Rough scale alignment
        scale1 = np.linalg.norm(partial.get_max_bound() - partial.get_min_bound()) / \
                np.linalg.norm(full.get_max_bound() - full.get_min_bound())
        scale2 = (bbox_volume(partial) / bbox_volume(full)) ** (1/3)
        scale3 = estimate_scale_from_nn(partial, full, k=1, top_k=100)
        scale_ratio = np.mean([scale1, scale2, scale3])
        if abs(scale_ratio - 1.0) > 0.05:
            full.scale(scale_ratio, center=full.get_center())
        else:
            print("Skipping scaling; size already aligned.")

    # Visual check after centering
    if debug:
        partial.paint_uniform_color([1.0, 0.0, 0.0])
        full.paint_uniform_color([0.0, 0.65, 0.93])
        draw(partial, full, title="Denoised")
        start = time.time()
    
    base_voxel = auto_voxel(partial)    
    voxel_size = base_voxel               
    if debug:
        print(f"voxel_size = {voxel_size:.4f} m")
        print("Running RANSAC …")

    # Down-sample + features
    f_down, f_fpfh = preprocess(full, voxel_size)
    p_down, p_fpfh = preprocess(partial, voxel_size)

    # RANSAC global registration
    iter_cap = auto_ransac_iters(len(f_down.points))
    best_res = None
    thresholds = [4,6,8]
    fitness_thresh=0.30
    rmse_thresh=0.02
    for coef in thresholds:
        max_dist = voxel_size * coef
        print(f"[RANSAC] max_corr = voxel*{coef:.1f} ({max_dist:.4f} m)")
        res = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            f_down, p_down, f_fpfh, p_fpfh,
            mutual_filter=True,
            max_correspondence_distance=max_dist,
            estimation_method=o3d.pipelines.registration.
                TransformationEstimationPointToPoint(with_scaling=False),
            ransac_n=3,
            criteria=o3d.pipelines.registration.
                RANSACConvergenceCriteria(iter_cap, 4_000)
        )
        print(f"fitness {res.fitness:.3f}, rmse {res.inlier_rmse:.4f}")
        if best_res is None or res.fitness > best_res.fitness:
            best_res = res
        if res.fitness >= fitness_thresh and res.inlier_rmse <= rmse_thresh:
            print("early-stop: good enough")
            break
    
    if debug:
        print(f'time cost for RANSAC: {time.time() -start}s')
        print("RANSAC transform:\n", best_res.transformation)
        full_ransac = copy.deepcopy(full).transform(best_res.transformation)
        partial.paint_uniform_color([1.0, 0.0, 0.0])      
        full_ransac.paint_uniform_color([0.0, 0.6, 1.0]) 
        draw(partial, full_ransac, title="After RANSAC")


    # Re-estimate normals and orient consistently
    for cloud in (full, partial):
        cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_size * 2, max_nn=30))
        cloud.orient_normals_consistent_tangent_plane(50)

    # ---------- ICP multi-scale & best-selection ----------
    current_T    = best_res.transformation
    # factors      = [7, 6, 5, 4, 3, 2, 1, 0.5, 0.25]
    factors      = [2.0, 1.0, 0.5]
    max_iter     = 30
    best_reg     = None
    best_score   = -np.inf       
    for factor in factors:
        max_dist = voxel_size * factor
        reg = o3d.pipelines.registration.registration_icp(
            source=full,
            target=partial,
            max_correspondence_distance=max_dist,
            init=current_T,
            estimation_method=o3d.pipelines.registration.
                TransformationEstimationPointToPlane(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=max_iter)
        )

        if debug:
            print(f"ICP @ {factor:4.2f}×voxel  fit={reg.fitness:.3f}  "
                f"rmse={reg.inlier_rmse:.4f}")

        denom = max_dist
        score = reg.fitness - 0.3 * (reg.inlier_rmse / denom)
        if score > best_score:
            best_score = score
            best_reg   = reg
        current_T = reg.transformation

    if best_reg is None:
        print("ICP failed: no acceptable solution")
        return
    result_icp = best_reg
    print("ICP best transform:\n", result_icp.transformation)

    # Transform full model
    full_aligned = copy.deepcopy(full).transform(result_icp.transformation)

    # Extract missing region
    distances = np.asarray(full_aligned.compute_point_cloud_distance(partial))
    missing_idx = np.where(distances > voxel_size * 2.0)[0]
    recovered = full_aligned.select_by_index(missing_idx)

    if debug:
        partial.paint_uniform_color([1.0, 0.0, 0.0])
        full_aligned.paint_uniform_color([0.0, 0.65, 0.93])
        draw(full_aligned, partial, title="ICP result")

    # Merge and save
    combined = (partial + recovered).voxel_down_sample(voxel_size * 0.8)
    return recovered


if __name__ == "__main__":
    # FULL_PATH = "/home/siyao/project/RoboCup/manipulation/3DSGrasp/src/3DSGrasp/Completion/dataset/3dsgrasp_ycb_train_test_split/input/starbucks_coffee_1.ply"
    #src/3DSGrasp/Completion/dataset/3dsgrasp_ycb_train_test_split/input/detergent_bottle_poisson_004
    #src/3DSGrasp/Completion/dataset/3dsgrasp_ycb_train_test_split/input/lime_poisson_001
    #src/3DSGrasp/Completion/dataset/3dsgrasp_ycb_train_test_split/input/jar_poisson_006
    # FULL_PATH = r"C:\Users\Administrator\Desktop\temp\_0_0_5_y.xyz"
    FULL_PATH = "/home/siyao/project/RoboCup/manipulation/3DSGrasp/src/3DSGrasp/Completion/dataset/3dsgrasp_ycb_train_test_split/gt/jar_poisson_006/test/_0_0_8_y.xyz"
    # PART_PATH = "/home/siyao/project/RoboCup/manipulation/3DSGrasp/src/3DSGrasp/Completion/dataset/3dsgrasp_ycb_train_test_split/input/jar_poisson_006/test/_0_0_8_x.xyz"
    PART_PATH = None
    VOXEL_FACTOR = 45
    recovered_pcl = pcl_registraion(FULL_PATH, PART_PATH, scale=False, debug=False)
