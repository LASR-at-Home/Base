#!/usr/bin/env python3
from copy import deepcopy
import numpy as np
import tf2_ros as tf
import ros_numpy as rnp

from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped


def pcl_transform(
    pcl: PointCloud2, transform: TransformStamped, target_frame: str = "map"
) -> PointCloud2:
    """Transforms a pointclound using a given transform message.
    Needed as the tf2 transform function returns an un-orderded pcl.
    Whilst we want an ordered pcl.

    Args:
        pcl (PointCloud2): source pointcloud to transform.
        transform (TransformStamped): transform to apply

    Returns:
        PointCloud2: transformed pointcloud
    """

    pcl_arr = deepcopy(rnp.point_cloud2.pointcloud2_to_array(pcl))

    translation = transform.transform.translation
    rotation_q = transform.transform.rotation

    rotation_matrix = R.from_quat(
        [rotation_q.x, rotation_q.y, rotation_q.z, rotation_q.w]
    )

    pcl_x = pcl_arr["x"]
    pcl_y = pcl_arr["y"]
    pcl_z = pcl_arr["z"]

    pcl_x_y_z_arr = np.array([pcl_x, pcl_y, pcl_z])

    C, H, W = pcl_x_y_z_arr.shape

    pcl_x_y_z_arr = pcl_x_y_z_arr.reshape(-1, H * W).T

    transformed_pcl = rotation_matrix.apply(pcl_x_y_z_arr) + np.array(
        [translation.x, translation.y, translation.z]
    )

    transformed_pcl = transformed_pcl.T.reshape(C, H, W)

    pcl_arr["x"] = transformed_pcl[0]
    pcl_arr["y"] = transformed_pcl[1]
    pcl_arr["z"] = transformed_pcl[2]

    transformed_pcl = rnp.point_cloud2.array_to_pointcloud2(
        pcl_arr, stamp=pcl.header.stamp, frame_id=target_frame
    )
    return transformed_pcl
