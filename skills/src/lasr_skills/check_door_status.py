#!/usr/bin/env python

import rospy
import smach
import smach_ros
import threading
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np


class CheckDoorStatus(smach.State):
    def __init__(self, estimated_depth=1.2, threshold=0.7):
        smach.State.__init__(self, outcomes=["open", "closed", "error"])
        self.lock = threading.Lock()
        self.depth_data = None
        self.ready = False
        self.sub = rospy.Subscriber("/xtion/depth_registered/points", PointCloud2, self.callback)
        self.estimated_depth = estimated_depth
        self.threshold = threshold

    def callback(self, msg):
        with self.lock:
            try:
                cloud = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)
                rospy.loginfo("Received %d points", len(cloud))
                rospy.loginfo("Sample points:\n%s", cloud[:5])

                # Widened ROI box
                mask = (
                    (cloud[:, 0] > 0.2) & (cloud[:, 0] < 0.7) &  # left-right (X)
                    (cloud[:, 1] > 0.0) & (cloud[:, 1] < 0.6) &  # up-down (Y)
                    (cloud[:, 2] > 0.3) & (cloud[:, 2] < 4.0)    # depth (Z)
                )
                roi_points = cloud[mask]
                rospy.loginfo("Filtered points in ROI: %d", roi_points.shape[0])

                if roi_points.shape[0] < 50:
                    rospy.logwarn("Too few points in ROI — assuming door is open.")
                    self.depth_data = "open"
                else:
                    avg_depth = np.mean(roi_points[:, 2])  # Z-axis is depth
                    rospy.loginfo("Avg object depth (Z): %.2f m", avg_depth)

                    if abs(avg_depth - self.estimated_depth) > self.threshold:
                        self.depth_data = "open"
                    else:
                        self.depth_data = "closed"

                self.ready = True
                self.sub.unregister()

            except Exception as e:
                rospy.logerr("Error processing point cloud: %s", e)
                self.depth_data = "error"
                self.ready = True
                self.sub.unregister()

    def execute(self, userdata):
        rospy.loginfo("Checking door status near estimated depth: %.2f m", self.estimated_depth)
        self.ready = False
        self.depth_data = None

        timeout = rospy.Time.now() + rospy.Duration(10)
        rate = rospy.Rate(5)
        while not self.ready and rospy.Time.now() < timeout:
            rate.sleep()

        if not self.ready:
            rospy.logerr("Timed out waiting for depth data.")
            return "error"

        return self.depth_data


def main():
    rospy.init_node("door_status_state_machine")

    sm = smach.StateMachine(outcomes=["DONE", "FAILED"])

    with sm:
        smach.StateMachine.add(
            "CHECK_DOOR_STATUS",
            CheckDoorStatus(estimated_depth=1.2, threshold=0.7),
            transitions={
                "open": "DONE",
                "closed": "DONE",
                "error": "FAILED",
            }
        )

    smach_viewer = smach_ros.IntrospectionServer("viewer", sm, "/SM_ROOT")
    smach_viewer.start()

    outcome = sm.execute()
    rospy.loginfo("State machine finished with outcome: %s", outcome)

    smach_viewer.stop()


if __name__ == "__main__":
    main()
