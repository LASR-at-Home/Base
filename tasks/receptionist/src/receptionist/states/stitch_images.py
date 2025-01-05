#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import smach
from receptionist.states import PointCloudSweep
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from stitching import Stitcher
from cv2_pcl import pcl_to_cv2
from cv2_img import cv2_img_to_msg

# TODO test main function and SM

class StitchImage(smach.State, Node):
    def __init__(self):
        Node.__init__(self, "stitch_image")
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["transformed_pointclouds"],
            output_keys=["stitched_image"],
        )
        self.stitcher = Stitcher()
        self.stitcher_pub = self.create_publisher(Image, "/vision/sitched_img", qos_profile=1)

    def execute(self, userdata):
        transformed_pointclouds = userdata.transformed_pointclouds
        if len(transformed_pointclouds) == 0:
            self.get_logger().error("No point clouds to stitch")
            return "failed"

        imgs = [pcl_to_cv2(pcl) for pcl in transformed_pointclouds]
        stitched_image = self.stitcher.stitch(imgs)
        self.stitcher_pub.publish(cv2_img_to_msg(stitched_image))
        userdata.stitched_image = stitched_image
        return "succeeded"

def main(args=None):
    rclpy.init(args=args)
    # rospy.init_node("stitch_images")
    sweep_points = [
        # Point(5.71, 3.06, 0.8),
        Point(6.50, 3.53, 0.8),
        Point(6.25, 3.53, 0.8),
        Point(5.5, 3.53, 0.8),
        Point(5.0, 3.53, 0.8),
        Point(4.96, 3.53, 0.8),
        Point(4.80, 3.73, 0.8),
    ]
    while rclpy.ok():
        node = rclpy.create_node("stitch_images")
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

        # TODO check the behaviour works as expected
        # wait for user to press enter before running again
        input("Press Enter to run again...")
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("Shutting down...")
        finally:
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
