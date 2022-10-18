#!/usr/bin/python3
import numpy as np

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