#!/usr/bin/env python3
from typing import List
import rospy
import smach
import numpy as np
from receptionist.states import PointCloudSweep
from geometry_msgs.msg import Polygon, Point, PointStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from stitching import Stitcher
from cv2_pcl import pcl_to_cv2
from cv2_img import cv2_img_to_msg


class StitchImage(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["transformed_pointclouds"],
            output_keys=["stitched_image"],
        )
        self.stitcher = Stitcher()
        self.sticher_pub = rospy.Publisher("/vision/sitched_img", Image, queue_size=1)

    def execute(self, userdata):
        transformed_pointclouds = userdata.transformed_pointclouds
        if len(transformed_pointclouds) == 0:
            rospy.logerr("No point clouds to stitch")
            return "failed"

        imgs = [pcl_to_cv2(pcl) for pcl in transformed_pointclouds]
        stitched_image = self.stitcher.stitch(imgs)
        self.sticher_pub.publish(cv2_img_to_msg(stitched_image))
        userdata.stitched_image = stitched_image
        return "succeeded"


if __name__ == "__main__":
    rospy.init_node("stitch_images")
    sweep_points = [
        # Point(5.71, 3.06, 0.8),
        Point(6.50, 3.53, 0.8),
        Point(6.25, 3.53, 0.8),
        Point(5.5, 3.53, 0.8),
        Point(5.0, 3.53, 0.8),
        Point(4.96, 3.53, 0.8),
        Point(4.80, 3.73, 0.8),
    ]
    while not rospy.is_shutdown():
        sm = smach.StateMachine(outcomes=["succeeded", "failed"])
        with sm:
            sm.userdata.transformed_pointclouds = []
            smach.StateMachine.add(
                "Sweep",
                PointCloudSweep(
                    sweep_points=sweep_points,
                ),
                transitions={
                    "succeeded": "StitchImages",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "StitchImages",
                StitchImage(),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )

        outcome = sm.execute()
        # wait for user to press enter before running again
        input("Press Enter to run again...")
    rospy.spin()
