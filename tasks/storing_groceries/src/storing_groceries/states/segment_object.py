import rospy
import numpy as np
import smach

from cv_bridge import CvBridge
import cv2_pcl
import cv2

import sensor_msgs.point_cloud2 as point_cloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header


class SegmentObject(smach.State):

    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["selected_object"],
            output_keys=["masked_cloud"],
        )
        self._cv_bridge = CvBridge()

    def execute(self, userdata):
        detection, pcl = userdata.selected_object

        im = cv2_pcl.pcl_to_cv2(pcl)

        contours = np.array(detection.xyseg).reshape(-1, 2)
        mask = np.zeros(shape=im.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, pts=[contours], color=255)
        indices = np.argwhere(mask)
        pcl_xyz = np.array(
            list(
                point_cloud2.read_points(
                    pcl, field_names=("x", "y", "z"), skip_nans=False
                )
            ),
            dtype=np.float32,
        ).reshape(im.shape[0], im.shape[1], 3)

        masked_points = pcl_xyz[indices[:, 0], indices[:, 1]]
        masked_points = masked_points[~np.isnan(masked_points).any(axis=1)]
        masked_points = masked_points[~np.all(masked_points == 0, axis=1)]

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = pcl.header.frame_id

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # pack the points into a list of tuples
        points_list = [tuple(p) for p in masked_points]

        userdata.masked_cloud = point_cloud2.create_cloud(header, fields, points_list)

        return "succeeded"
