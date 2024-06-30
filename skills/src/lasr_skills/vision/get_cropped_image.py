#!/usr/bin/env python3
from typing import Optional, List
import numpy as np
import rospy
import smach
import cv2
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
        model_2d: str = "yolov8x-seg.pt",
        model_3d: str = "yolov8x-seg.pt",
        use_mask: bool = True,
        confidence_2d: float = 0.5,
        confidence_3d: float = 0.5,
        nmsthresh_2d: float = 0.3,
        nmsthresh_3d: float = 0.3,
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

            model_2d (str, optional): Name of the YOLO model to use for 2D detection.

            model_3d (str, optional): Name of the YOLO model to use for 3D detection.

            use_mask (bool, optional): Whether to use the mask to crop the image. Defaults to True.

            confidence_2d (float, optional): Confidence threshold for 2D detection.

            confidence_3d (float, optional): Confidence threshold for 3D detection.

            nmsthresh_2d (float, optional): Non-maximum suppression threshold for 2D detection.

            nmsthresh_3d (float, optional): Non-maximum suppression threshold for 3D detection.
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
        self._model_2d = model_2d
        self._model_3d = model_3d
        self._use_mask = use_mask
        self._confidence_2d = confidence_2d
        self._confidence_3d = confidence_3d
        self._nmsthresh_2d = nmsthresh_2d
        self._nmsthresh_3d = nmsthresh_3d
        self._yolo_2d = rospy.ServiceProxy("/yolov8/detect", YoloDetection)
        self._yolo_3d = rospy.ServiceProxy("/yolov8/detect3d", YoloDetection3D)
        self._robot_pose_topic = "/robot_pose"

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

    def _2d_bbox_crop(
        self, image: np.ndarray, detections: List[Detection]
    ) -> np.ndarray:
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

        if len(detections) == 0:
            raise ValueError("No detections found")

        detection = min(
            detections,
            key=lambda det: np.sqrt(
                (x_to_compare - det.xywh[0]) ** 2 + (y_to_compare - det.xywh[1]) ** 2
            ),
        )

        x, y, w, h = (
            detection.xywh[0],
            detection.xywh[1],
            detection.xywh[2],
            detection.xywh[3],
        )
        return image[y - h // 2 : y + h // 2, x - w // 2 : x + w // 2]

    def _2d_mask_crop(
        self, image: np.ndarray, detections: List[Detection]
    ) -> np.ndarray:
        """Crops the image to the according to the desired crop_method using a mask.

        Args:
            image (np.ndarray): Image to crop
            detections (YoloDetection): YOLO Detections of the desired object
            in the image.
        Returns:
            np.ndarray: Cropped image
        """

        # Keeping this in a separate function as might want to make function
        # more complex, i.e., add noise to other detections rather than filling
        # in the whole image, etc.

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

        if len(detections) == 0:
            raise ValueError("No detections found")

        if len(detections[0].xyseg) == 0:
            raise ValueError("No segmentation found")

        detection_index = min(
            range(len(detections)),
            key=lambda i: np.sqrt(
                (x_to_compare - detections[i].xywh[0]) ** 2
                + (y_to_compare - detections[i].xywh[1]) ** 2
            ),
        )

        # x,y coords of the detection
        # Taken from https://stackoverflow.com/questions/37912928/fill-the-outside-of-contours-opencv
        mask = np.array(detections[detection_index].xyseg).reshape(-1, 2)
        stencil = np.zeros(image.shape).astype(image.dtype)
        colour = (255, 255, 255)
        cv2.fillPoly(stencil, [mask], colour)
        # Bitwise AND with 0s is 0s, hence we get the image only where the mask is
        # with black elsewhere.
        result = cv2.bitwise_and(image, stencil)

        return result

    def _3d_bbox_crop(
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
        closest_detection = min(
            detections,
            key=lambda det: np.sqrt(
                (robot_loc.x - det.point.x) ** 2
                + (robot_loc.y - det.point.y) ** 2
                + (robot_loc.z - det.point.z) ** 2
            ),
        )
        furthest_detection = max(
            detections,
            key=lambda det: np.sqrt(
                (robot_loc.x - det.point.x) ** 2
                + (robot_loc.y - det.point.y) ** 2
                + (robot_loc.z - det.point.z) ** 2
            ),
        )

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

    def _3d_mask_crop(
        self, pointcloud: np.ndarray, robot_loc: Point, detections: List[Detection3D]
    ) -> np.ndarray:
        """Crops the image to the according to the desired crop_method using a mask.

        Args:
            pointcloud (np.ndarray): Image to crop
            detections (YoloDetection): YOLO Detections of the desired object
            in the image.
        Returns:
            np.ndarray: Cropped image
        """
        closest_detection_index = min(
            range(len(detections)),
            key=lambda i: np.sqrt(
                (robot_loc.x - detections[i].point.x) ** 2
                + (robot_loc.y - detections[i].point.y) ** 2
                + (robot_loc.z - detections[i].point.z) ** 2
            ),
        )
        furthest_detection_index = max(
            range(len(detections)),
            key=lambda i: np.sqrt(
                (robot_loc.x - detections[i].point.x) ** 2
                + (robot_loc.y - detections[i].point.y) ** 2
                + (robot_loc.z - detections[i].point.z) ** 2
            ),
        )

        if self._crop_method == "closest":
            detection_index = closest_detection_index
        elif self._crop_method == "furthest":
            detection_index = furthest_detection_index
        else:
            raise ValueError(f"Invalid 3D crop_method: {self._crop_method}")

        rgb_image = pcl_to_cv2(pointcloud)

        # x,y coords of the detection
        # Taken from https://stackoverflow.com/questions/37912928/fill-the-outside-of-contours-opencv
        mask = np.array(detections[detection_index].xyseg).reshape(-1, 2)
        stencil = np.zeros(rgb_image.shape).astype(rgb_image.dtype)
        colour = (255, 255, 255)
        cv2.fillPoly(stencil, [mask], colour)
        # Bitwise AND with 0s is 0s, hence we get the image only where the mask is
        # with black elsewhere.
        result = cv2.bitwise_and(rgb_image, stencil)

        return result

    def execute(self, userdata):
        try:
            if self._crop_method in self._valid_2d_methods:
                img_msg = rospy.wait_for_message(self._rgb_topic, Image)
                detections = self._yolo_2d(
                    img_msg, self._model_2d, self._confidence_2d, self._nmsthresh_2d
                ).detected_objects
                detections = [
                    det for det in detections if det.name == self._object_name
                ]
                img_cv2 = msg_to_cv2_img(img_msg)
                cropped_image = (
                    self._2d_mask_crop(img_cv2, detections)
                    if self._use_mask
                    else self._2d_bbox_crop(img_cv2, detections)
                )
            elif self._crop_method in self._valid_3d_methods:
                pointcloud_msg = rospy.wait_for_message(self._depth_topic, PointCloud2)
                robot_loc = rospy.wait_for_message(
                    self._robot_pose_topic, PoseWithCovarianceStamped
                ).pose.pose.position
                detections = self._yolo_3d(
                    pointcloud_msg,
                    self._model_3d,
                    self._confidence_3d,
                    self._nmsthresh_3d,
                ).detected_objects
                detections = [
                    det for det in detections if det.name == self._object_name
                ]
                cropped_image = (
                    self._3d_mask_crop(pointcloud_msg, robot_loc, detections)
                    if self._use_mask
                    else self._3d_bbox_crop(pointcloud_msg, robot_loc, detections)
                )
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
        get_cropped_image = GetCroppedImage(
            "person",
            crop_method="closest",
            use_mask=True,
        )
        sm = smach.StateMachine(outcomes=["succeeded", "failed"])
        with sm:
            smach.StateMachine.add(
                "GET_CROPPED_IMAGE",
                get_cropped_image,
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
        sm.execute()
        input("Press Enter to continue...")
