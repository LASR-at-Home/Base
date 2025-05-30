#!/usr/bin/env python

import rospy
import smach
import smach_ros
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np


class CheckDoorStatus(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["open", "closed", "error"])
        self.sub = rospy.Subscriber("/xtion/depth_registered/points", PointCloud2, self.callback)
        self.result_ready = False
        self.result = None

    def callback(self, msg):
        if self.result_ready:
            return  # Already processed

        try:
            cloud_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)

            # ROI in camera frame
            mask = (
                (cloud_array[:, 2] > 1.2) & (cloud_array[:, 2] < 1.6) &   # Z: depth
                (cloud_array[:, 0] > 0.4) & (cloud_array[:, 0] < 0.6) &   # X: side
                (cloud_array[:, 1] > 0.1) & (cloud_array[:, 1] < 0.3)     # Y: down
            )

            roi_points = cloud_array[mask]
            if roi_points.shape[0] < 100:
                rospy.logwarn_throttle(5, "Not enough points in ROI")
                self.result = "open"
            else:
                avg_depth = np.mean(roi_points[:, 2])
                rospy.loginfo("Avg object depth (Z): %.2f m", avg_depth)
                self.result = "closed" if avg_depth < 1.4 else "open"

            self.result_ready = True
            self.sub.unregister()  # stop further callbacks

        except Exception as e:
            rospy.logerr("Error in point cloud processing: %s", e)
            self.result = "error"
            self.result_ready = True
            self.sub.unregister()

    def execute(self, userdata):
        rospy.loginfo("Waiting for depth data...")
        timeout = rospy.Time.now() + rospy.Duration(5)

        while not self.result_ready and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

        if self.result_ready:
            return self.result
        else:
            rospy.logerr("Timeout waiting for depth data")
            return "error"


def main():
    rospy.init_node("check_door_status_state_machine")

    sm = smach.StateMachine(outcomes=["DONE", "FAILED"])

    with sm:
        smach.StateMachine.add(
            "CHECK_DOOR_STATUS",
            CheckDoorStatus(),
            transitions={
                "open": "DONE",
                "closed": "DONE",
                "error": "FAILED"
            }
        )

    smach_viewer = smach_ros.IntrospectionServer("viewer", sm, "/SM_ROOT")
    smach_viewer.start()

    outcome = sm.execute()
    rospy.loginfo("Finished with outcome: %s", outcome)

    smach_viewer.stop()


if __name__ == "__main__":
    main()
