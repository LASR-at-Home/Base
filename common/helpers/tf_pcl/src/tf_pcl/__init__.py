#!/usr/bin/env python3
from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from sensor_msgs_py import point_cloud2


def pcl_transform(
    pcl: PointCloud2, transform: TransformStamped, target_frame: str = "map"
) -> PointCloud2:
    """Transforms a point cloud using a given transform message."""
    pcl_arr = deepcopy(
        list(point_cloud2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=True))
    )

    translation = transform.transform.translation
    rotation_q = transform.transform.rotation

    rotation_matrix = R.from_quat(
        [rotation_q.x, rotation_q.y, rotation_q.z, rotation_q.w]
    )

    pcl_x_y_z_arr = np.array(pcl_arr)

    # Apply rotation and translation
    transformed_pcl = rotation_matrix.apply(pcl_x_y_z_arr) + np.array(
        [translation.x, translation.y, translation.z]
    )

    # Create a new PointCloud2 message
    transformed_pcl_msg = point_cloud2.create_cloud(
        pcl.header, pcl.fields, transformed_pcl
    )
    return transformed_pcl_msg