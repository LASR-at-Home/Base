#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from sensor_msgs_py import point_cloud2
from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation as R


class PointCloudTransformer(Node):
    def __init__(self):
        super().__init__("pointcloud_transformer")
        self.subscription = self.create_subscription(
            PointCloud2,
            "input_pointcloud",
            self.pcl_callback,
            10,
        )
        self.publisher = self.create_publisher(PointCloud2, "output_pointcloud", 10)

    def pcl_callback(self, pcl: PointCloud2):
        # Example transform (identity transform for demonstration purposes)
        transform = TransformStamped()
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        transformed_pcl = pcl_transform(pcl, transform)
        self.publisher.publish(transformed_pcl)


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


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
