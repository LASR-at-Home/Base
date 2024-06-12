#!/usr/bin/env python3
from typing import Optional, List
import numpy as np
import cv2
import rospy
import smach
from sensor_msgs.msg import Image
from lasr_vision_msgs.msg import Detection
from lasr_vision_msgs.srv import YoloDetection, YoloDetection3D
from cv2_img import cv2_img_to_msg, msg_to_cv2_img


class GetCroppedImage(smach.State):
    def __init__(
        self,
        object_name: str,
        crop_method: str = "centered",
        debug_publisher: str = "/skills/get_cropped_image/debug",
        rgb_topic: Optional[str] = "/xtion/rgb/image_raw",
        depth_topic: Optional[str] = "/xtion/depth_registered/points",
    ):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["cropped_image"],
        )
        """_summary_

        Args:
            crop_method (str, optional): _description_. Defaults to "centered".
        """
        self._crop_method = crop_method
        self._debug_pub = rospy.Publisher(debug_publisher, Image, queue_size=1)
        self._rgb_topic = rgb_topic
        self._depth_topic = depth_topic
        self._object_name = object_name
        self._yolo_2d = rospy.ServiceProxy("/yolov8/detect", YoloDetection)
        self._yolo_3d = rospy.ServiceProxy("/yolov8/detect3d", YoloDetection3D)

        self._valid_2d_methods = [
            "centered",
            "left-most",
            "right-most",
            "top-most",
            "bottom-most",
        ]
        self._valid_3d_methods = ["closest", "furthest", "polygon"]

        if crop_method in self._valid_2d_methods:
            self._yolo_2d.wait_for_service()
        elif crop_method in self._valid_3d_methods:
            self._yolo_3d.wait_for_service()
        else:
            raise ValueError(f"Invalid crop_method: {crop_method}")

    def _2d_crop(self, image: np.ndarray, detections: List[Detection]) -> np.ndarray:
        """Crops the image to the desired object that is closest to the
        centroid of the image.

        Args:
            image (np.ndarray): Image to crop
            detections (YoloDetection): Yolo Detections of the desired object
            in the image.
        Returns:
            np.ndarray: Cropped image
        """
        if self._crop_method == "centered":
            y_to_compare = image.shape[0] // 2
            x_to_compare = image.shape[1] // 2
        elif self._crop_method == "right-most":
            x_to_compare = 0
            y_to_compare = image.shape[0] // 2
        elif self._crop_method == "left-most":
            x_to_compare = image.shape[1]
            y_to_compare = image.shape[0] // 2
        elif self._crop_method == "top-most":
            x_to_compare = image.shape[1] // 2
            y_to_compare = 0
        elif self._crop_method == "bottom-most":
            x_to_compare = image.shape[1] // 2
            y_to_compare = image.shape[0]
        else:
            raise ValueError(f"Invalid 2D crop_method: {self._crop_method}")

        detection = None
        dist = float("inf")

        for det in detections:
            x, y, _, _ = det.xywh[0], det.xywh[1], det.xywh[2], det.xywh[3]
            det_center_x = x
            det_center_y = y
            det_dist = np.sqrt(
                (x_to_compare - det_center_x) ** 2 + (y_to_compare - det_center_y) ** 2
            )
            if det_dist < dist:
                dist = det_dist
                detection = det

        if detection is None:
            raise ValueError("No detections found")

        x, y, w, h = (
            detection.xywh[0],
            detection.xywh[1],
            detection.xywh[2],
            detection.xywh[3],
        )
        return image[y - h // 2 : y + h // 2, x - w // 2 : x + w // 2]

    def execute(self, userdata):
        if self._crop_method in self._valid_2d_methods:
            img_msg = rospy.wait_for_message(self._rgb_topic, Image)
            detections = self._yolo_2d(img_msg, "yolov8x.pt", 0.5, 0.3).detected_objects
            detections = [det for det in detections if det.name == self._object_name]
            img_cv2 = msg_to_cv2_img(img_msg)
            cropped_image = self._2d_crop(img_cv2, detections)
            cropped_msg = cv2_img_to_msg(cropped_image)
            self._debug_pub.publish(cropped_msg)
            userdata.cropped_image = cropped_msg
            return "succeeded"
        elif self._crop_method in self._valid_3d_methods:
            pass
        return "failed"


if __name__ == "__main__":
    rospy.init_node("get_cropped_image")
    while not rospy.is_shutdown():
        get_cropped_image = GetCroppedImage(
            "bottle", crop_method="top-most", rgb_topic="/usb_cam/image_raw"
        )
        sm = smach.StateMachine(outcomes=["succeeded", "failed"])
        with sm:
            smach.StateMachine.add(
                "GET_CROPPED_IMAGE",
                get_cropped_image,
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
        sm.execute()
        # wait for user to press enter
        input("Press Enter to continue...")
