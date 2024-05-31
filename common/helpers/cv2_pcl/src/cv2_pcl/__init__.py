import numpy as np
from sensor_msgs.msg import PointCloud2
import ros_numpy as rnp
import cv2
from cv2_img import cv2_img_to_msg

from typing import Tuple, Union

Mat = np.ndarray

def pcl_to_img_msg(pcl: PointCloud2) -> Mat:
    """
    Convert a given PointCloud2 message to img_msg
    """
    # keep the same timestamp
    cv2 = pcl_to_cv2(pcl)

    return cv2_img_to_msg(cv2, pcl.header.stamp)



def pcl_to_cv2(
    pcl: PointCloud2, height: Union[int, None] = None, width: Union[int, None] = None
) -> Mat:
    """
    Convert a given PointCloud2 message to a cv2 image
    """

    height = height or pcl.height
    width = width or pcl.width

    # Extract rgb image from pointcloud
    frame = np.fromstring(pcl.data, dtype=np.uint8)
    frame = frame.reshape(height, width, -1)
    frame = frame[:, :, 16:19]

    # Ensure array is contiguous
    frame = np.ascontiguousarray(frame, dtype=np.uint8)

    return frame


def seg_to_centroid(
    pcl: PointCloud2,
    xyseg: np.ndarray,
    height: Union[int, None] = None,
    width: Union[int, None] = None,
) -> np.ndarray:
    """
    Computes the centroid of a given segment in a pointcloud
    """

    height = height or pcl.height
    width = width or pcl.width

    # Convert xyseg to contours
    contours = xyseg.reshape(-1, 2)

    # cv2 image
    cv_im = pcl_to_cv2(pcl, height, width)

    # Compute mask from contours
    mask = np.zeros(shape=cv_im.shape[:2])
    cv2.fillPoly(mask, pts=[contours], color=(255, 255, 255))

    # Extract mask indices from bounding box
    indices = np.argwhere(mask)

    if indices.shape[0] == 0:
        return np.full(3, np.nan)

    # Unpack pointcloud into xyz array
    pcl_xyz = rnp.point_cloud2.pointcloud2_to_xyz_array(pcl, remove_nans=False)
    pcl_xyz = pcl_xyz.reshape(height, width, 3)

    # Extract points of interest
    xyz_points = [pcl_xyz[x][y] for x, y in indices]

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
    Computes the centroid of a given bounding box in a pointcloud
    """

    height = height or pcl.height
    width = width or pcl.width

    # Convert xywh to bounding box coordinates.
    x1, y1, x2, y2 = x, y, x + w, y + h

    # cv2 image
    frame = pcl_to_cv2(pcl, height, width)
    # Compute mask from bounding box
    mask = np.zeros(shape=frame.shape[:2])
    mask[y1:y2, x1:x2] = 255  # bounding box dimensions

    # Extract mask indices from bounding box
    indices = np.argwhere(mask)
    if indices.shape[0] == 0:
        return np.full(3, np.nan)

    # Unpack pointcloud into xyz array
    pcl_xyz = rnp.point_cloud2.pointcloud2_to_xyz_array(pcl, remove_nans=False)
    pcl_xyz = pcl_xyz.reshape(height, width, 3)

    # Extract points of interest
    xyz_points = [pcl_xyz[x][y] for x, y in indices]

    # Get mean (centroid) point.
    return np.nanmean(xyz_points, axis=0)
