#!/usr/bin/env python

import rospy
import smach
import smach_ros
import threading
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np


class CheckDoorStatus(smach.State):
    def __init__(self, expected_closed_depth=1.2, roi=None, change_thresh=0.5, open_thresh=0.6):
        smach.State.__init__(self, outcomes=["open", "closed", "error"])
        self.expected_depth = expected_closed_depth
        self.change_thresh = change_thresh
        self.open_thresh = open_thresh

        # ROI boundaries (X, Y, Z) in meters
        self.roi = roi if roi else {
            'x': (0.2, 0.7),
            'y': (0.0, 0.6),
            'z': (0.3, 4.0)
        }

        self.lock = threading.Lock()
        self.latest_cloud = None
        self.ready = False

        self.sub = rospy.Subscriber("/xtion/depth_registered/points", PointCloud2, self.callback)

    def callback(self, msg):
        with self.lock:
            self.latest_cloud = msg
            self.ready = True

    def get_avg_depth(self):
        with self.lock:
            if self.latest_cloud is None:
                return None

            try:
                cloud = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.latest_cloud, remove_nans=True)
                mask = (
                    (cloud[:, 0] > self.roi['x'][0]) & (cloud[:, 0] < self.roi['x'][1]) &
                    (cloud[:, 1] > self.roi['y'][0]) & (cloud[:, 1] < self.roi['y'][1]) &
                    (cloud[:, 2] > self.roi['z'][0]) & (cloud[:, 2] < self.roi['z'][1])
                )
                roi_points = cloud[mask]
                if roi_points.shape[0] < 50:
                    rospy.logwarn("Not enough points in ROI")
                    return None

                avg_depth = np.mean(roi_points[:, 2])
                rospy.loginfo("Avg ROI depth: %.2f m", avg_depth)
                return avg_depth

            except Exception as e:
                rospy.logerr("Point cloud processing error: %s", e)
                return None

    def execute(self, userdata):
        rospy.loginfo("Waiting for initial depth reading...")
        self.ready = False
        self.latest_cloud = None

        timeout = rospy.Time.now() + rospy.Duration(5)
        rate = rospy.Rate(5)
        while not self.ready and rospy.Time.now() < timeout:
            rate.sleep()

        initial_depth = self.get_avg_depth()
        if initial_depth is None:
            return "error"

        # Check if already open compared to expected closed depth
        if abs(initial_depth - self.expected_depth) > self.open_thresh:
            rospy.loginfo("Initial depth already differs from expected closed depth → door is open")
            return "open"

        rospy.loginfo("Waiting for second depth to check for changes...")
        rospy.sleep(3.0)  # wait for user or environment to change door

        second_depth = self.get_avg_depth()
        if second_depth is None:
            return "error"

        # Check if depth has changed significantly
        if abs(second_depth - initial_depth) > self.change_thresh:
            rospy.loginfo("Depth changed → door is now open")
            return "open"

        rospy.loginfo("Depth stayed similar → door is closed")
        return "closed"


def main():
    rospy.init_node("door_status_state_machine")

    sm = smach.StateMachine(outcomes=["DONE", "FAILED"])

    with sm:
        smach.StateMachine.add(
            "CHECK_DOOR_STATUS",
            CheckDoorStatus(
                expected_closed_depth=1.2,  # adjust for cabinet (~0.5) or room door (~1.2)
                change_thresh=0.4,
                open_thresh=0.6
            ),
            transitions={
                "open": "DONE",
                "closed": "DONE",
                "error": "FAILED",
            }
        )

    smach_viewer = smach_ros.IntrospectionServer("viewer", sm, "/SM_ROOT")
    smach_viewer.start()
    outcome = sm.execute()
    rospy.loginfo("SM finished with outcome: %s", outcome)
    smach_viewer.stop()


if __name__ == "__main__":
    main()
