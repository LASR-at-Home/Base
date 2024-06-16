#!/usr/bin/env python3
from typing import Optional, List
import numpy as np
import rospy
import smach
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from lasr_vision_msgs.msg import Detection, Detection3D
from lasr_vision_msgs.srv import YoloDetection, YoloDetection3D
from cv2_img import cv2_img_to_msg, msg_to_cv2_img
from cv2_pcl import pcl_to_cv2


class GetCroppedImage(smach.State):
    def __init__(
        self,
        object_name: str,
        crop_method: str = "centered",
        debug_publisher: str = "/skills/get_cropped_image/debug",
        rgb_topic: Optional[str] = "/xtion/rgb/image_raw",
        depth_topic: Optional[str] = "/xtion/depth_registered/points",
    ):
        """This skill gets a semantically cropped image of a given object, allowing
        you to specify whether you want the nearest object, the furthest object,
        the most centered object, etc.

        Args:
            object_name (str): YOLO class name of object to detect.

            crop_method (str, optional): Which semantic crop method to use. See the valid method
            variables for the list of options. Defaults to "centered".

            debug_publisher (str, optional): Name of the topic to publish the cropped images to.
            Defaults to "/skills/get_cropped_image/debug".

            rgb_topic (Optional[str], optional): Name of the topic to get RGB images from.
            Defaults to "/xtion/rgb/image_raw".

            depth_topic (Optional[str], optional): Name of the topic to get RGBD images from.
            Defaults to "/xtion/depth_registered/points".
        """
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["img_msg"],
        )

        self._crop_method = crop_method
        self._debug_pub = rospy.Publisher(debug_publisher, Image, queue_size=1)
        self._rgb_topic = rgb_topic
        self._depth_topic = depth_topic
        self._object_name = object_name
        self._yolo_2d = rospy.ServiceProxy("/yolov8/detect", YoloDetection)
        self._yolo_3d = rospy.ServiceProxy("/yolov8/detect3d", YoloDetection3D)
        self._robot_pose_topic = "/amcl_pose"

        self._valid_2d_methods = [
            "centered",
            "left-most",
            "right-most",
            "top-most",
            "bottom-most",
        ]
        self._valid_3d_methods = ["closest", "furthest"]

        if crop_method in self._valid_2d_methods:
            self._yolo_2d.wait_for_service()
        elif crop_method in self._valid_3d_methods:
            self._yolo_3d.wait_for_service()
        else:
            raise ValueError(f"Invalid crop_method: {crop_method}")

    def _2d_crop(self, image: np.ndarray, detections: List[Detection]) -> np.ndarray:
        """Crops the image to the according to the desired crop_method.

        Args:
            image (np.ndarray): Image to crop
            detections (YoloDetection): YOLO Detections of the desired object
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
            det_center_x, det_center_y = det.xywh[0], det.xywh[1]
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

    def _3d_crop(
        self, pointcloud: np.ndarray, robot_loc: Point, detections: List[Detection3D]
    ) -> np.ndarray:
        """Crops the image to the desired object that is closest to the
        centroid of the image.

        Args:
            pointcloud (np.ndarray): pointcloud from depth camera; RGB will be extracted
            from this.
            robot_loc (Point): Current location of the robot, used to calculate
            3D-distance.
            detections (YoloDetection): Yolo Detections of the desired object
            in the image.
        Returns:
            np.ndarray: Cropped image
        """
        min_dist = float("inf")
        max_dist = 0
        closest_detection = None
        furthest_detection = None
        for det in detections:
            dist = np.sqrt(
                (robot_loc.x - det.point.x) ** 2
                + (robot_loc.y - det.point.y) ** 2
                + (robot_loc.z - det.point.z) ** 2
            )
            if dist < min_dist:
                min_dist = dist
                closest_detection = det
            if dist > max_dist:
                max_dist = dist
                furthest_detection = det

        if self._crop_method == "closest":
            detection = closest_detection
        elif self._crop_method == "furthest":
            detection = furthest_detection
        else:
            raise ValueError(f"Invalid 3D crop_method: {self._crop_method}")

        if detection is None:
            raise ValueError(f"No detection found")

        rgb_image = pcl_to_cv2(pointcloud)
        x, y, w, h = (
            detection.xywh[0],
            detection.xywh[1],
            detection.xywh[2],
            detection.xywh[3],
        )

        return rgb_image[y - h // 2 : y + h // 2, x - w // 2 : x + w // 2]

    def execute(self, userdata):
        try:
            if self._crop_method in self._valid_2d_methods:
                img_msg = rospy.wait_for_message(self._rgb_topic, Image)
                detections = self._yolo_2d(
                    img_msg, "yolov8x.pt", 0.5, 0.3
                ).detected_objects
                detections = [
                    det for det in detections if det.name == self._object_name
                ]
                img_cv2 = msg_to_cv2_img(img_msg)
                cropped_image = self._2d_crop(img_cv2, detections)
            elif self._crop_method in self._valid_3d_methods:
                pointcloud_msg = rospy.wait_for_message(self._depth_topic, PointCloud2)
                robot_loc = rospy.wait_for_message(
                    self._robot_pose_topic, PoseWithCovarianceStamped
                ).pose.pose.position
                detections = self._yolo_3d(
                    pointcloud_msg, "yolov8x-seg.pt", 0.5, 0.3
                ).detected_objects
                detections = [
                    det for det in detections if det.name == self._object_name
                ]
                cropped_image = self._3d_crop(pointcloud_msg, robot_loc, detections)
            else:
                raise ValueError(f"Invalid crop_method: {self._crop_method}")
            cropped_msg = cv2_img_to_msg(cropped_image)
            self._debug_pub.publish(cropped_msg)
            userdata.img_msg = cropped_msg
            return "succeeded"
        except Exception as e:
            rospy.logerr(e)
            return "failed"


if __name__ == "__main__":
    rospy.init_node("get_cropped_image")
    while not rospy.is_shutdown():
        get_cropped_image = GetCroppedImage("chair", crop_method="furthest")
        sm = smach.StateMachine(outcomes=["succeeded", "failed"])
        with sm:
            smach.StateMachine.add(
                "GET_CROPPED_IMAGE",
                get_cropped_image,
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
        sm.execute()
        input("Press Enter to continue...")
