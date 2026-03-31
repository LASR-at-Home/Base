#!/usr/bin/env python3
from copy import deepcopy
import numpy as np
import ros2_numpy as rnp

from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped


def pcl_transform(
    pcl: PointCloud2, transform: TransformStamped, target_frame: str = "map"
) -> PointCloud2:
    """Transforms a pointclound using a given transform message.
    
    Args:
        pcl (PointCloud2): source pointcloud to transform.
        transform (TransformStamped): transform to apply
        target_frame (str): The new frame_id for the output header.

    Returns:
        PointCloud2: transformed pointcloud
    """

    pcl_arr = deepcopy(rnp.point_cloud2.pointcloud2_to_array(pcl))

    original_shape = pcl_arr.shape

    xyz = np.array([pcl_arr["x"], pcl_arr["y"], pcl_arr["z"]])

    xyz_flat = xyz.reshape(3, -1)

    translation = transform.transform.translation
    rotation_q = transform.transform.rotation
    rotation_matrix = R.from_quat(
        [rotation_q.x, rotation_q.y, rotation_q.z, rotation_q.w]
    )
    translation_vec = np.array([translation.x, translation.y, translation.z])

    transformed_xyz = rotation_matrix.apply(xyz_flat.T) + translation_vec

    transformed_xyz = transformed_xyz.T

    pcl_arr["x"] = transformed_xyz[0].reshape(original_shape)
    pcl_arr["y"] = transformed_xyz[1].reshape(original_shape)
    pcl_arr["z"] = transformed_xyz[2].reshape(original_shape)

    transformed_pcl = rnp.point_cloud2.array_to_pointcloud2(
        pcl_arr, stamp=pcl.header.stamp, frame_id=target_frame
    )
    return transformed_pcl