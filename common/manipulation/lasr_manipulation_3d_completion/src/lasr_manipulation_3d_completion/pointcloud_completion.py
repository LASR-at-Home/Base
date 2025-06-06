#!/usr/bin/env python3

import torch
import numpy as np
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from lasr_manipulation_3d_completion.sgrasp_model import SGrasp
from argparse import Namespace


def build_and_load_model(
    ckpt_path,
    name,
    num_pred,
    num_query,
    knn_layer,
    trans_dim,
    map_location='cpu'
):
    if name != 'SGrasp':
        raise ValueError(f"Unsupported model type: {name}")

    config = Namespace(
        num_pred=num_pred,
        num_query=num_query,
        knn_layer=knn_layer,
        trans_dim=trans_dim,
    )
    model = SGrasp(config)

    model.to(map_location)

    if not os.path.exists(ckpt_path):
        raise FileNotFoundError(f"Checkpoint not found: {ckpt_path}")

    ckpt = torch.load(ckpt_path, map_location=map_location)

    if 'model' in ckpt:
        state_dict = ckpt['model']
    elif 'base_model' in ckpt:
        state_dict = ckpt['base_model']
    else:
        raise RuntimeError("Checkpoint must contain 'model' or 'base_model'.")

    # Remove 'module.' if present
    state_dict = {k.replace("module.", ""): v for k, v in state_dict.items()}
    model.load_state_dict(state_dict)
    model.eval()

    return model


def farthest_point_sample(point, npoint):
    """
    Input:
        point: (N, D)
    Return:
        sampled: (npoint, D)
    """
    N, D = point.shape
    xyz = point[:, :3]
    centroids = np.zeros((npoint,), dtype=np.int32)
    distance = np.ones((N,)) * 1e10
    farthest = np.random.randint(0, N)
    for i in range(npoint):
        centroids[i] = farthest
        centroid = xyz[farthest]
        dist = np.sum((xyz - centroid) ** 2, axis=1)
        distance = np.minimum(dist, distance)
        farthest = np.argmax(distance)
    return point[centroids]


def preprocess_pointcloud(points: np.ndarray, target_num: int = 2048):
    """
    Make sure input point cloud has target_num points.
    """
    if points.shape[0] < target_num:
        multiplier = target_num // points.shape[0] + 1
        points = np.tile(points, (multiplier, 1))[:target_num]
    return farthest_point_sample(points, target_num)


def complete_pointcloud(model, partial_np: np.ndarray, device='cuda') -> np.ndarray:
    """
    Use pretrained model to complete the partial point cloud.
    """
    if partial_np.ndim != 2 or partial_np.shape[1] != 3:
        raise ValueError("Input point cloud must be of shape (N, 3)")

    # Normalize
    centroid = partial_np.mean(axis=0)
    pc = partial_np - centroid
    m = np.max(np.linalg.norm(pc, axis=1))
    pc = pc / m

    # Convert to tensor
    pc_tensor = torch.tensor(pc, dtype=torch.float32).unsqueeze(0).to(device)

    with torch.no_grad():
        _, dense_points = model(pc_tensor)

    dense_points = dense_points.squeeze(0).cpu().numpy()

    # Unnormalize
    return dense_points * (m + m / 6) + centroid
