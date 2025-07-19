import os
import smach
import rospy
import cv2
import numpy as np


from typing import Optional, List

from geometry_msgs.msg import PoseWithCovarianceStamped
from lasr_vision_msgs.msg import Detection3D
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class CropImage3D(smach.State):
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
            dict: Dictionary of cropped detections with class names as keys. If no detections
            match the filters, the dictionary will contain None for those classes. Values
            are Lists of dictionaries with keys "cropped_image" and "detection_3d", sorted according
            to crop logic, where "cropped_image" is a ROS Image message and "detection_3d" is the
            corresponding Detection3D message.
        """
        self.robot_pose_topic = robot_pose_topic
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["detections_3d", "image_raw"],
            output_keys=["cropped_detections"],
        )
        self.filters = filters or []
        self.crop_logic = crop_logic
        self.crop_type = crop_type
        self._bridge = CvBridge()

        self.debug_publisher = rospy.Publisher(
            "/skills/crop_image_3d/debug", Image, queue_size=1
        )

        if self.crop_type not in ["masked", "bbox"]:
            raise ValueError(
                f"Invalid crop_type: {self.crop_type}. Must be 'masked' or 'bbox'."
            )
        if self.crop_logic not in ["nearest", "farthest"]:
            raise ValueError(
                f"Invalid crop_logic: {self.crop_logic}. Must be 'nearest' or 'farthest'."
            )

    def execute(self, userdata):
        detections = userdata["detections_3d"]
        if not detections:
            rospy.logwarn("No 3D detections found.")
            return "failed"
        robot_pose_msg = rospy.wait_for_message(
            self.robot_pose_topic, PoseWithCovarianceStamped
        )
        # Pose in map frame, same as detected objects
        robot_x, robot_y, robot_z = (
            robot_pose_msg.pose.pose.position.x,
            robot_pose_msg.pose.pose.position.y,
            robot_pose_msg.pose.pose.position.z,
        )

        rgb_image = self._bridge.imgmsg_to_cv2(
            userdata["image_raw"], desired_encoding="rgb8"
        )

        if self.filters:
            detections = [det for det in detections if det.name in self.filters]
            if not detections:
                rospy.logwarn("No detections match the specified filters.")
                return "failed"

        # Sort detections based on the crop logic
        eucl_dist = lambda det: (
            det.point.x - robot_x,
            det.point.y - robot_y,
            det.point.z - robot_z,
        )
        reverse = self.crop_logic == "farthest"
        detections.sort(key=eucl_dist, reverse=reverse)

        cropped_detections = {k: None for k in self.filters}
        for det in detections:

            if self.crop_type == "masked":
                # x,y coords of the detection
                # Taken from https://stackoverflow.com/questions/37912928/fill-the-outside-of-contours-opencv
                mask = np.array(det.xyseg).reshape(-1, 2)
                stencil = np.zeros(rgb_image.shape).astype(rgb_image.dtype)
                colour = (255, 255, 255)
                cv2.fillPoly(stencil, [mask], colour)
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

            masked_img_msg = self._bridge.cv2_to_imgmsg(masked_image, encoding="rgb8")
            if cropped_detections[det.name]:
                cropped_detections[det.name].append(
                    {
                        "cropped_image": masked_img_msg,
                        "detection_3d": det,
                    }
                )
            else:
                cropped_detections[det.name] = [
                    {
                        "cropped_image": masked_img_msg,
                        "detection_3d": det,
                    }
                ]

        # Convert image to ROS Image message for debugging
        if cropped_detections:
            all_cropped_images = [
                cropped_detections[det_name]["cropped_image"]
                for det_name in cropped_detections
                if cropped_detections[det_name] is not None
            ]
            debug_image_msg = self._bridge.cv2_to_imgmsg(
                all_cropped_images[0], encoding="rgb8"
            )
            self.debug_publisher.publish(debug_image_msg)

        userdata["cropped_detections"] = cropped_detections

        return "succeeded"


if __name__ == "__main__":
    from lasr_skills import Detect3D

    rospy.init_node("crop_image_3d")
    while not rospy.is_shutdown():
        crop = CropImage3D()
        detect = Detect3D()
        sm = smach.StateMachine(outcomes=["succeeded", "failed"])
        with sm:
            smach.StateMachine.add(
                "DETECT_3D",
                detect,
                transitions={"succeeded": "CROP_IMAGE_3D", "failed": "failed"},
            )
            smach.StateMachine.add(
                "CROP_IMAGE_3D",
                crop,
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )

        outcome = sm.execute()
        rospy.loginfo(f"SMACH execution outcome: {outcome}")
        rospy.spin()
        input("Press Enter to run again or Ctrl+C to exit...")
