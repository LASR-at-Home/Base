#!/usr/bin/env python3
import rospy
import numpy as np
from unsafe_traversal.srv import LaserDist, LaserDistResponse
from sensor_msgs.msg import LaserScan
import math

class LaserDistCheckerService:
    """A ROS service that returns the mean distance of a laser scan within a specific field of view and with outliers removed."""

    def __init__(self):
        """Initialize the service."""
        self.laser_topic = "/scan"
        self.laser_dist_srv = rospy.Service("/unsafe_traversal/laser_dist_checker", LaserDist, self.laser_dist)

    def remove_outliers(self, data, threshold=1.5):
        """
        Remove outliers from the data using the interquartile range method.

        Args:
            data (np.ndarray): An array of data points.
            threshold (float): The number of interquartile ranges beyond which a data point is considered an outlier.

        Returns:
            np.ndarray: The data with outliers removed.
        """
        q1, q3 = np.nanpercentile(data, [25, 75])
        iqr = q3 - q1
        lower_bound = q1 - threshold * iqr
        upper_bound = q3 + threshold * iqr
        data = data[(data >= lower_bound) & (data <= upper_bound)]
        return data

    def nanmean_without_outliers(self, data, threshold=1.5):
        """
        Calculate the mean of the data points with outliers removed.

        Args:
            data (np.ndarray): An array of data points.
            threshold (float): The number of interquartile ranges beyond which a data point is considered an outlier.

        Returns:
            float: The mean of the data points with outliers removed.
        """
        return np.nanmean(self.remove_outliers(data, threshold))

    def filter_by_fov_degrees(self, scan, fov):
        """
        Filter a laser scan by a specific field of view.

        Args:
            scan (LaserScan): The laser scan message to be filtered.
            fov (float): The field of view in degrees.

        Returns:
            LaserScan: The filtered laser scan message.
        """
        filtered_scan = LaserScan()
        
        filtered_scan.header = scan.header
        filtered_scan.angle_min = scan.angle_min
        filtered_scan.angle_max = scan.angle_max
        filtered_scan.scan_time = scan.scan_time
        filtered_scan.range_min = scan.range_min
        filtered_scan.range_max = scan.range_max

        fov_half = int((math.radians(fov) / 2) / scan.angle_increment)
        center_idx = len(scan.ranges) // 2
        fov_start = center_idx - fov_half
        fov_end = center_idx + fov_half

        filtered_scan.ranges = [np.nan] * len(scan.ranges)
        filtered_scan.ranges[fov_start:fov_end+1] = scan.ranges[fov_start:fov_end+1]

        return filtered_scan

    def laser_dist(self, req):
        """
        Handle a LaserDist service request.

        Args:
            req (LaserDistRequest): The service request.

        Returns:
            LaserDistResponse: The service response with the mean distance of the laser scan within the specified field of view.
        """
        scan = rospy.wait_for_message(self.laser_topic, LaserScan)
        filtered_scan = self.filter_by_fov_degrees(scan, req.fov_degrees)
        mean_distance = self.nanmean_without_outliers(np.array(filtered_scan.ranges))
        return LaserDistResponse(mean_distance)