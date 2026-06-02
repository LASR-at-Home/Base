import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

import yasmin
from yasmin import State, StateMachine
import yasmin_ros
from yasmin_ros import set_ros_loggers

# import os
import cv2
import numpy as np

from typing import Optional, List

from geometry_msgs.msg import PoseWithCovarianceStamped

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class CropImage3D(State):
    def __init__(
        self,
        robot_pose_topic: str = "/amcl_pose",
        filters: Optional[List[str]] = None,
        crop_logic: str = "nearest",
        crop_type: str = "masked",
    ):
        """Returns cropped RGB images based on 3D detections. For example, cropping the RGB
        image around the closest person to the robot.


        Args:
            node (rclpy.Node): a rclpy Node.

            robot_pose_topic (str, optional): Topic to get the map frame position of the robot
            . Defaults to "/amcl_pose".

            filters (Optional[List[str]], optional): List of YOLO class names to filter.
            Defaults to None.

            crop_logic (str, optional): Nearest/farthest crop logic. Defaults to "nearest".

            crop_type (str, optional): masked or bbox. Defaults to "masked". If mask, uses
            the segmentation mask of the YOLO detection to crop the image. If bbox, uses the
            bounding box from YOLO instead (i.e., if not using a YOLO segmentation model).

        Returns:
            (in userdata["cropped_images"]):
            dict: Dictionary of cropped images with class names as keys. If no detections
            match the filters, the dictionary will contain None for those classes.
        """
        self.robot_pose_topic = robot_pose_topic
        super().__init__(outcomes=["succeeded", "failed"])

        self.add_input_key("detections_3d")
        self.add_input_key("image_raw")

        self.add_output_key("cropped_images")

        self.filters = filters
        self.crop_logic = crop_logic
        self.crop_type = crop_type
        self._bridge = CvBridge()

        self.debug_publisher = yasmin_ros.logger_node.create_publisher(
            Image,
            "/skills/crop_image_3d/debug",
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
        )

        if self.crop_type not in ["masked", "bbox"]:
            raise ValueError(
                f"Invalid crop_type: {self.crop_type}. Must be 'masked' or 'bbox'."
            )
        if self.crop_logic not in ["nearest", "farthest"]:
            raise ValueError(
                f"Invalid crop_logic: {self.crop_logic}. Must be 'nearest' or 'farthest'."
            )

    def execute(self, blackboard):
        detections = blackboard["detections_3d"].detected_objects
        if not detections:
            yasmin.YASMIN_LOG_WARN("No 3D detections found.")
            return "failed"

        # From: https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/wait_for_message.py
        success, robot_pose_msg = wait_for_message(
            PoseWithCovarianceStamped, yasmin_ros.logger_node, self.robot_pose_topic
        )
        if not success:
            yasmin.YASMIN_LOG_WARN("Timed out waiting for robot pose.")
            return "failed"

        # Pose in map frame, same as detected objects
        robot_x, robot_y, robot_z = (
            robot_pose_msg.pose.pose.position.x,
            robot_pose_msg.pose.pose.position.y,
            robot_pose_msg.pose.pose.position.z,
        )

        rgb_image = self._bridge.imgmsg_to_cv2(
            blackboard["image_raw"], desired_encoding="rgb8"
        )

        # If there are filters keep only those detections
        if self.filters:
            detections = [det for det in detections if det.name in self.filters]
            if not detections:
                yasmin.YASMIN_LOG_WARN(
                    "No detections match the specified filters."
                )
                return "failed"

        # Sort detections based on the crop logic
        eucl_dist = lambda det: (
            det.point.x - robot_x,
            det.point.y - robot_y,
            det.point.z - robot_z,
        )
        reverse = self.crop_logic == "farthest"
        detections.sort(key=eucl_dist, reverse=reverse)

        if self.filters:
            cropped_images = {
                k: None for k in self.filters
            }  # Place holder for cropped images
        else:
            cropped_images = {det.name: None for det in detections}

        for det in detections:
            # Already have the closest/farthest detection for this class
            if cropped_images[det.name] is not None:
                continue

            if self.crop_type == "masked":
                # x,y coords of the detection
                yasmin.YASMIN_LOG_INFO(f"Processing {det.name}:")

                if len(det.xyseg) == 0:
                    yasmin.YASMIN_LOG_WARN(
                        f"No segmentation data for {det.name}, skipping"
                    )
                    continue
                # Taken from https://stackoverflow.com/questions/37912928/fill-the-outside-of-contours-opencv
                mask = np.array(det.xyseg, dtype=np.int32).reshape(-1, 2)
                stencil = np.zeros(rgb_image.shape).astype(rgb_image.dtype)
                colour = (255, 255, 255)
                cv2.fillPoly(stencil, [mask], colour)
                yasmin.YASMIN_LOG_INFO(
                    f"  stencil filled pixels: {np.count_nonzero(stencil)} / {stencil.size}"
                )

                # Bitwise AND with 0s is 0s, hence we get the image only where the mask is
                # with black elsewhere.
                masked_image = cv2.bitwise_and(rgb_image, stencil)

            elif self.crop_type == "bbox":
                x, y, w, h = (
                    det.xywh[0],
                    det.xywh[1],
                    det.xywh[2],
                    det.xywh[3],
                )
                masked_image = rgb_image[
                    y - h // 2 : y + h // 2, x - w // 2 : x + w // 2
                ]

            cropped_images[det.name] = masked_image

        # Convert image to ROS Image message for debugging
        debug_image = next(
            (img for img in cropped_images.values() if img is not None), None
        )
        if debug_image is not None:
            # debug_image = next(iter(cropped_images.values()))
            debug_image_msg = self._bridge.cv2_to_imgmsg(debug_image, encoding="rgb8")
            self.debug_publisher.publish(debug_image_msg)

        blackboard["cropped_images"] = cropped_images

        return "succeeded"


def main():
    from lasr_skills import Detect3D

    rclpy.init()
    set_ros_loggers()

    try:
        sm = StateMachine(outcomes=["succeeded", "failed"], handle_sigint=True)
        sm.add_state(
            "DETECT_3D",
            Detect3D(),
            transitions={"succeeded": "CROP_IMAGE_3D", "failed": "failed"},
        )
        sm.add_state(
            "CROP_IMAGE_3D",
            CropImage3D(),
            transitions={"succeeded": "succeeded", "failed": "failed"},
        )

        outcome = sm.execute()
        yasmin.YASMIN_LOG_INFO(f"SMACH execution outcome: {outcome}")

    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)
    
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
