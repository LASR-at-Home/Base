#!/usr/bin/python
import numpy as np
import rospy
import numpy as np
# import ros_numpy as rnp
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
from common_math.srv import TfTransform, TfTransformRequest

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
    frame = frame[:,:,16:19]

    # Ensure array is contiguous
    frame = np.ascontiguousarray(frame, dtype=np.uint8)

    return frame


def tf_transform(target_frame, pose_array=None, pointcloud=None, point=None):
    """
    Transforms the given message to the target frame.
    Parameters:
        target_frame {frame_id} -- frame to transform to
        pose_array {PoseArray} -- array of poses
        pointcloud {PointCloud2}
        point {PointStamped}

    Returns:
        response {TfTransformResponse} -- target_pose_array {PoseArray}, target_pointcloud {PointCloud2}, target_point {PointStamped}
    """

    assert pose_array is not None or pointcloud is not None or point is not None

    rospy.wait_for_service('tf_transform', timeout=10)
    try:
        tf_transform_srv = rospy.ServiceProxy('tf_transform', TfTransform)
        request = TfTransformRequest()
        if pose_array is not None:
            request.pose_array = pose_array
        if pointcloud is not None:
            request.pointcloud = pointcloud
        if point is not None:
            request.point = point
        request.target_frame.data = target_frame
        response = tf_transform_srv(request)
        # print('transform done!')
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
def bb_to_centroid(pcl_msg, x, y, w, h):
    """
    Computes a centroid in the map plane from a given bounding box.
    Performs this by extracting the bounding box from the pointcloud,
    and computing the centroid from that.

    Parameters
    ----------
    pcl_msg : sensor_msgs/PointCloud2
        Input pointcloud (organised).
    x : float
        Bottom left x of bounding box.
    y : float
        Bottom left y of bounding box.
    w : float
        Width of bounding box.
    h : float
        Height of bounding box.

    Returns
    -------
    geometry_msgs/PointStamped : centroid
    """

    # Convert xywh to bounding box coordinates.
    x1, y1, x2, y2 = x, y, x + w, y + h

    # cv2 image
    frame = pcl_msg_to_cv2(pcl_msg)

    # Compute mask from bounding box
    mask = np.zeros(shape=frame.shape[:2])
    mask[y1:y2, x1:x2] = 255  # bounding box dimensions

    # Extract mask indices from bounding box
    indices = np.argwhere(mask)
    if len(indices) < 1:
        return None
    pcl_xyz = rnp.point_cloud2.pointcloud2_to_xyz_array(pcl_msg, remove_nans=False)

    # Create x, y, z array, for indexing.
    xyz_points = []
    for x, y in indices:
        x, y, z = pcl_xyz[x][y]
        xyz_points.append([x, y, z])

    # Get mean (centroid) point.
    x, y, z = np.nanmean(xyz_points, axis=0)

    # Construct PointStamped message.
    centroid_ = PointStamped(point=Point(x, y, z), header=Header(1, rospy.Time(0), "xtion_depth_optical_frame"))

    # Transform to map frame
    centroid_ = tf_transform('map', point=centroid_).target_point

    return centroid_