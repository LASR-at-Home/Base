#!/usr/bin/python3
import numpy as np
import rospy
import numpy as np
import ros_numpy as rnp
# import cv2
# from geometry_msgs.msg import PointStamped, Point
# from std_msgs.msg import Header
# from robocup_receptionist.srv import TfTransform, TfTransformRequest
import math
import cv2



def pcl_msg_to_cv2(pcl_msg):
    """
    Constructs a cv2 image from a PointCloud2 message.

    Parameters
    ----------
    pcl_msg : sensor_msgs/PointCloud2
        Input pointcloud (organised)

    Returns
    -------
    np.array : cv2 image
    """

    # Extract rgb image from pointcloud
    frame = np.fromstring(pcl_msg.data, dtype=np.uint8)
    frame = frame.reshape(pcl_msg.height, pcl_msg.width, 32)
    frame = frame[:, :, 16:19]

    # Ensure array is contiguous
    frame = np.ascontiguousarray(frame, dtype=np.uint8)

    return frame


def seg_to_centroid(pcl_msg, xyseg):
    # Convert xyseg to contours
    contours = xyseg.reshape(-1, 2)

    # cv2 image
    cv_im = pcl_msg_to_cv2(pcl_msg)
    # Compute mask from contours
    mask = np.zeros(shape=cv_im.shape[:2])
    cv2.fillPoly(mask, pts=[contours], color=(255, 255, 255))

    # Extract mask indices from bounding box
    indices = np.argwhere(mask)

    if indices.shape[0] == 0:
        return np.full(3, np.nan)

    # Unpack pointcloud into xyz array
    pcl_xyz = rnp.point_cloud2.pointcloud2_to_xyz_array(pcl_msg, remove_nans=False)

    # Extract points of interest
    xyz_points = [pcl_xyz[x][y] for x, y in indices]

    return np.nanmean(xyz_points, axis=0)


def bb_to_centroid(pcl_msg, x, y, w, h):
    # Convert xywh to bounding box coordinates.
    x1, y1, x2, y2 = x, y, x + w, y + h

    # cv2 image
    frame = pcl_msg_to_cv2(pcl_msg)
    # Compute mask from bounding box
    mask = np.zeros(shape=frame.shape[:2])
    mask[y1:y2, x1:x2] = 255  # bounding box dimensions

    # Extract mask indices from bounding box
    indices = np.argwhere(mask)
    if indices.shape[0] == 0:
        return np.full(3, np.nan)

    # Unpack pointcloud into xyz array
    pcl_xyz = rnp.point_cloud2.pointcloud2_to_xyz_array(pcl_msg, remove_nans=False)

    # Extract points of interest
    xyz_points = [pcl_xyz[x][y] for x, y in indices]

    # Get mean (centroid) point.
    return np.nanmean(xyz_points, axis=0)


def euclidian(a, b):
    '''
    Get the Euclidean distance between two positions
    '''
    return math.sqrt(
        math.pow(a.x - b.x, 2) +
        math.pow(a.y - b.y, 2) +
        math.pow(a.z - b.z, 2)
    )
def euclidian_distance(p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        a = np.array((x1, y1))
        b = np.array((x2, y2))
        return np.linalg.norm(a - b)
