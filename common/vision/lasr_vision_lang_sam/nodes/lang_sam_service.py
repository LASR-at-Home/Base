#!/usr/bin/env python3
"""
LangSAM Service for ROS - Provides language-guided segmentation capabilities.
This service can be configured to run with or without GPU acceleration.
"""

import os
import sys
from typing import List, Optional


# CRITICAL: Configure GPU before importing any deep learning libraries
def configure_gpu_from_args():
    """Configure GPU settings from command line arguments before any imports."""
    if len(sys.argv) > 1:
        try:
            use_gpu = bool(int(sys.argv[1]))
            if not use_gpu:
                os.environ["CUDA_VISIBLE_DEVICES"] = ""
                print(f"[LangSAM] GPU disabled - CUDA_VISIBLE_DEVICES set to empty")
                return False
            else:
                print(f"[LangSAM] GPU enabled")
                return True
        except (ValueError, IndexError):
            print(f"[LangSAM] Invalid GPU argument, defaulting to GPU enabled")
            return True
    print(f"[LangSAM] No GPU argument provided, defaulting to GPU enabled")
    return True


# Configure GPU settings immediately
GPU_ENABLED = configure_gpu_from_args()

# Now safe to import deep learning libraries
import rospy
import cv2
import cv2_img
import numpy as np
from PIL import Image

from lang_sam import LangSAM
from lang_sam.utils import draw_image
from lasr_tf.srv import TransformPoint, TransformPointRequest
from lasr_vision_msgs.srv import LangSam, LangSamRequest, LangSamResponse
from lasr_vision_msgs.msg import LangSamDetection
from sensor_msgs.msg import Image as SensorImage
from geometry_msgs.msg import Point, PointStamped


class LangSamService:
    """
    ROS service wrapper for LangSAM (Language-guided Segment Anything Model).

    This service processes images with text prompts to generate segmentation masks
    and 3D point clouds using depth information.
    """

    def __init__(self):
        """Initialize the LangSAM service."""
        self.debug_publisher = rospy.Publisher(
            "/lasr_vision/lang_sam/debug", SensorImage, queue_size=10
        )

        rospy.loginfo(
            f"Initializing LangSAM with GPU {'enabled' if GPU_ENABLED else 'disabled'}"
        )
        self._model = LangSAM()
        self._service = rospy.Service("/lasr_vision/lang_sam", LangSam, self._lang_sam)
        self._tf_service = rospy.ServiceProxy(
            "/tf_server/transform_point", TransformPoint
        )
        self._tf_service.wait_for_service()
        rospy.loginfo("/lasr_vision/lang_sam service is ready!")

    def _imgmsg_to_cv2(self, img_msg) -> np.ndarray:
        """
        Convert ROS Image message to OpenCV format.

        Args:
            img_msg: ROS sensor_msgs/Image message

        Returns:
            np.ndarray: OpenCV image array
        """
        n_channels = 1
        dtype = np.dtype(np.float32)
        dtype = dtype.newbyteorder(">" if img_msg.is_bigendian else "<")

        img_buf = (
            np.asarray(img_msg.data, dtype=dtype)
            if isinstance(img_msg.data, list)
            else img_msg.data
        )

        if n_channels == 1:
            im = np.ndarray(
                shape=(img_msg.height, int(img_msg.step / dtype.itemsize)),
                dtype=dtype,
                buffer=img_buf,
            )
            im = np.ascontiguousarray(im[: img_msg.height, : img_msg.width])
        else:
            im = np.ndarray(
                shape=(
                    img_msg.height,
                    int(img_msg.step / dtype.itemsize / n_channels),
                    n_channels,
                ),
                dtype=dtype,
                buffer=img_buf,
            )
            im = np.ascontiguousarray(im[: img_msg.height, : img_msg.width, :])

        # Handle byte order differences between message and system
        if img_msg.is_bigendian == (sys.byteorder == "little"):
            im = im.byteswap().newbyteorder()

        return im

    def _lang_sam(self, request: LangSamRequest) -> LangSamResponse:
        """
        Process image and text prompt to generate segmentation and 3D points.

        Args:
            request: LangSamRequest containing image, depth, and processing parameters

        Returns:
            LangSamResponse: List of detected objects with segmentation masks and 3D points
        """
        prompt = request.prompt
        sensor_image = request.image_raw

        # Extract camera intrinsic parameters
        K = request.depth_camera_info.K
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]

        # Convert ROS image messages to appropriate formats
        cv_im = cv2_img.msg_to_cv2_img(sensor_image)
        pil_image = Image.fromarray(cv_im)
        depth_image = self._imgmsg_to_cv2(request.depth_image)

        # Run LangSAM inference
        results = self._model.predict(
            [pil_image],
            [prompt],
            box_threshold=request.box_threshold,
            text_threshold=request.text_threshold,
        )

        response_results: List[LangSamDetection] = []

        for result in results:
            no_of_detections = len(result["masks"])
            if no_of_detections == 0:
                rospy.logwarn("No detections found for prompt: '%s'", prompt)
                continue

            # Publish debug visualization
            image_arr = np.array(pil_image)
            debug_image = draw_image(
                image_arr,
                result["masks"],
                result["boxes"],
                result["scores"],
                result["labels"],
            )
            debug_image_msg = cv2_img.cv2_img_to_msg(debug_image)
            self.debug_publisher.publish(debug_image_msg)

            # Process each detection
            for det in range(no_of_detections):
                try:
                    rounded_xyxy = [round(coord) for coord in result["boxes"][det]]
                    xywh = [
                        rounded_xyxy[0],
                        rounded_xyxy[1],
                        rounded_xyxy[2] - rounded_xyxy[0],
                        rounded_xyxy[3] - rounded_xyxy[1],
                    ]

                    # Extract 3D point from depth image using segmentation mask
                    mask = result["masks"][det]
                    v, u = np.where(mask)
                    z = depth_image[v, u]
                    valid = z > 0

                    if not np.any(valid):
                        rospy.logwarn(
                            "No valid depth points found for detection %d", det
                        )
                        continue

                    z = z[valid]
                    u = u[valid]
                    v = v[valid]

                    # Convert to 3D coordinates
                    x = z * (u - cx) / fx
                    y = z * (v - cy) / fy
                    points = np.stack((x, y, z), axis=1)
                    x, y, z = np.median(points, axis=0)

                    # Transform point to target frame
                    point = Point(x, y, z)
                    point_stamped = PointStamped()
                    point_stamped.header = request.depth_image.header
                    point_stamped.point = point

                    tf_response = self._tf_service(
                        TransformPointRequest(
                            input_point_stamped=point_stamped,
                            target_frame=request.target_frame,
                        )
                    )
                    detection_point = tf_response.transformed_point_stamped.point

                    # Handle mask scores (can be 0D array for single detection)
                    mask_score = (
                        result["mask_scores"][det]
                        if no_of_detections > 1
                        else float(result["mask_scores"])
                    )

                    response_results.append(
                        LangSamDetection(
                            xywh=xywh,
                            seg_mask=mask.flatten().astype(int).tolist(),
                            detection_score=result["scores"][det],
                            seg_mask_score=mask_score,
                            point=detection_point,
                        )
                    )

                except Exception as e:
                    rospy.logerr("Failed to process detection %d: %s", det, str(e))
                    continue

        return LangSamResponse(detections=response_results)


def main():
    """Main entry point for the LangSAM service."""
    rospy.init_node("lasr_vision_lang_sam")

    # Create service instance
    lang_sam_service = LangSamService()

    rospy.loginfo("LangSAM service initialized successfully")
    rospy.spin()


if __name__ == "__main__":
    main()
