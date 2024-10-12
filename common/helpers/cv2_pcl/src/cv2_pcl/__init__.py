import numpy as np
import struct
from sensor_msgs.msg import PointCloud2
import cv2
from cv2_img import cv2_img_to_msg
from typing import Tuple, Union

# ROS2-specific imports
import rclpy
from rclpy.node import Node

Mat = np.ndarray

def pointcloud2_to_xyz_array(pointcloud: PointCloud2, remove_nans=True):
    """
    Convert a sensor_msgs/PointCloud2 message to an Nx3 NumPy array.
    :param pointcloud: ROS2 PointCloud2 message.
    :param remove_nans: If True, NaN values will be removed.
    :return: Nx3 NumPy array of XYZ points.
    """
    fmt = _get_struct_fmt(pointcloud)
    width, height = pointcloud.width, pointcloud.height
    unpacker = struct.Struct(fmt)
    points = []

    for i in range(height * width):
        point_data = pointcloud.data[i * pointcloud.point_step:(i + 1) * pointcloud.point_step]
        point = unpacker.unpack(point_data[:12])  # Assuming XYZ are first 12 bytes (3 floats)
        points.append(point)

    # Convert to a NumPy array
    points = np.array(points)

    if remove_nans:
        points = points[~np.isnan(points).any(axis=1)]
    
    return points

def _get_struct_fmt(cloud_msg: PointCloud2):
    """
    Generate the struct format string from the PointCloud2 fields.
    :param cloud_msg: ROS2 PointCloud2 message.
    :return: Struct format string for unpacking the data.
    """
    # Define the data structure format string (assuming XYZ are all float32)
    fmt = 'fff'  # XYZ are three 32-bit floats (4 bytes each)
    return fmt

def pcl_to_img_msg(pcl: PointCloud2) -> Mat:
    """
    Convert a given PointCloud2 message to img_msg.
    """
    # keep the same timestamp
    cv2_img = pcl_to_cv2(pcl)

    return cv2_img_to_msg(cv2_img, pcl.header.stamp)

def pcl_to_cv2(
    pcl: PointCloud2, height: Union[int, None] = None, width: Union[int, None] = None
) -> Mat:
    """
    Convert a given PointCloud2 message to a cv2 image.
    """
    height = height or pcl.height
    width = width or pcl.width

    # Extract XYZ points and pack as an image (for example, RGB image representation)
    xyz_array = pointcloud2_to_xyz_array(pcl)

    # Placeholder for converting XYZ to RGB or any other visualization.
    # For example, scale and shift XYZ to [0, 255] for visualization as an image.
    frame = (xyz_array[:, :3] - np.min(xyz_array[:, :3])) / (np.max(xyz_array[:, :3]) - np.min(xyz_array[:, :3]))
    frame = (frame * 255).astype(np.uint8)

    # Reshape into a height x width x 3 image.
    frame = frame.reshape((height, width, 3))

    return frame

def seg_to_centroid(
    pcl: PointCloud2,
    xyseg: np.ndarray,
    height: Union[int, None] = None,
    width: Union[int, None] = None,
) -> np.ndarray:
    """
    Computes the centroid of a given segment in a pointcloud.
    """
    height = height or pcl.height
    width = width or pcl.width

    # Convert xyseg to contours
    contours = xyseg.reshape(-1, 2)

    # Convert PointCloud2 to NumPy array
    pcl_xyz = pointcloud2_to_xyz_array(pcl, remove_nans=False)
    pcl_xyz = pcl_xyz.reshape(height, width, 3)

    # Compute mask from contours
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(mask, pts=[contours], color=(255))

    # Extract mask indices
    indices = np.argwhere(mask)

    if indices.shape[0] == 0:
        return np.full(3, np.nan)

    # Extract points of interest based on mask
    xyz_points = [pcl_xyz[x, y] for x, y in indices]

    # Compute the centroid of the points
    return np.nanmean(xyz_points, axis=0)

def bb_to_centroid(
    pcl: PointCloud2,
    x: int,
    y: int,
    w: int,
    h: int,
    height: Union[int, None] = None,
    width: Union[int, None] = None,
) -> np.ndarray:
    """
    Computes the centroid of a given bounding box in a pointcloud.
    """
    height = height or pcl.height
    width = width or pcl.width

    # Convert PointCloud2 to NumPy array
    pcl_xyz = pointcloud2_to_xyz_array(pcl, remove_nans=False)
    pcl_xyz = pcl_xyz.reshape(height, width, 3)

    # Bounding box indices
    x1, y1, x2, y2 = x, y, x + w, y + h

    # Extract points in the bounding box
    xyz_points = pcl_xyz[y1:y2, x1:x2].reshape(-1, 3)

    if xyz_points.shape[0] == 0:
        return np.full(3, np.nan)

    # Compute the centroid of the points
    return np.nanmean(xyz_points, axis=0)
