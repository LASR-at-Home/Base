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
import rclpy
import cv2
import cv2_img
import numpy as np
from PIL import Image

from rclpy.node import Node

from lang_sam import LangSAM
from lang_sam.utils import draw_image
from lasr_tf.srv import TransformPoint
from lasr_vision_interfaces.srv import LangSam
from lasr_vision_interfaces.msg import LangSamDetection
from sensor_msgs.msg import Image as SensorImage
from geometry_msgs.msg import Point, PointStamped


class LangSamService(Node):

    _model: LangSAM
    debug_publisher: rclpy.publisher.Publisher

    def __init__(self):
        """Initialize the LangSAM service."""
        super().__init__("lasr_vision_lang_sam")

        self.debug_publisher = self.create_publisher(
            SensorImage, "/lasr_vision/lang_sam/debug", 10
        )

        self.get_logger().info(
            f"Initializing LangSAM with GPU {'enabled' if GPU_ENABLED else 'disabled'}"
        )
        self._model = LangSAM()
        self._service = self.create_service(LangSam, "/lasr_vision/lang_sam", self._lang_sam)
        self._tf_service = self.create_client(TransformPoint, "/tf_server/transform_point")
        while not self._tf_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /tf_server/transform_point ...")
        self.get_logger().info("/lasr_vision/lang_sam service is ready!")

    def _imgmsg_to_cv2(self, img_msg):
        # From
        # https://github.com/ros-perception/vision_opencv/blob/rolling/cv_bridge/python/cv_bridge/core.py

        # Software License Agreement (BSD License)
        #
        # Copyright (c) 2011, Willow Garage, Inc.
        # Copyright (c) 2016, Tal Regev.
        # Copyright (c) 2018 Intel Corporation.
        # All rights reserved.
        #
        # Redistribution and use in source and binary forms, with or without
        # modification, are permitted provided that the following conditions
        # are met:
        #
        #  * Redistributions of source code must retain the above copyright
        #    notice, this list of conditions and the following disclaimer.
        #  * Redistributions in binary form must reproduce the above
        #    copyright notice, this list of conditions and the following
        #    disclaimer in the documentation and/or other materials provided
        #    with the distribution.
        #  * Neither the name of Willow Garage, Inc. nor the names of its
        #    contributors may be used to endorse or promote products derived
        #    from this software without specific prior written permission.
        #
        # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
        # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
        # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
        # FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
        # COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
        # INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
        # BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
        # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
        # CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
        # LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
        # ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
        # POSSIBILITY OF SUCH DAMAGE.
        ####################################################################
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

        # If the byte order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == "little"):
            im = im.byteswap().newbyteorder()

        return im

    def _lang_sam(self, request: LangSam.Request, response: LangSam.Response) -> LangSam.Response:
        """Processes a given input image and text prompt to return a list of
        segmentation and bboxes using Grounded SAM.

        Args:
            request (LangSamRequest): Request with fields:
                - image_raw (sensor_msgs/Image): The input image to process.
                - prompt (str): The text prompt to guide the segmentation.
        Returns:
            LangSamResponse: Response with fields:
                - detections (List[LangSamDetection]): List of detected objects with their bounding boxes
                and segmentation masks.
        """

        prompt = request.prompt
        sensor_image = request.image_raw
        K = request.depth_camera_info.k
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]

        # Convert sensor_msgs/Image to PIL Image
        cv_im = cv2_img.msg_to_cv2_img(sensor_image)
        pil_image = Image.fromarray(cv_im)

        # rospy.loginfo(f"Raw depth image: {request.depth_image}")
        depth_image = self._imgmsg_to_cv2(request.depth_image)
        # rospy.loginfo(f"Processed depth image: {depth_image}")

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
                self.get_logger().warn("No detections found.")
                continue
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

            for det in range(no_of_detections):
                rounded_xyxy = [round(coord) for coord in result["boxes"][det]]
                # Convert xyxy to xywh format
                xywh = [
                    rounded_xyxy[0],
                    rounded_xyxy[1],
                    rounded_xyxy[2] - rounded_xyxy[0],
                    rounded_xyxy[3] - rounded_xyxy[1],
                ]
                mask = result["masks"][det]
                v, u = np.where(mask)
                z = depth_image[v, u]
                valid = z > 0
                z = z[valid]
                u = u[valid]
                v = v[valid]
                x = z * (u - cx) / fx
                y = z * (v - cy) / fy
                points = np.stack((x, y, z), axis=1)
                x, y, z = np.median(points, axis=0)

                point = Point(x=float(x), y=float(y), z=float(z))
                point_stamped = PointStamped()
                point_stamped.header = request.depth_image.header
                point_stamped.point = point

                tf_req = TransformPoint.Request()
                tf_req.input_point_stamped = point_stamped
                tf_req.target_frame = request.target_frame
                future = self._tf_service.call_async(tf_req)
                rclpy.spin_until_future_complete(self, future)
                tf_response = future.result()
                point_transformed = tf_response.transformed_point_stamped

                detection_point = point_transformed.point

                response_results.append(
                    LangSamDetection(
                        xywh=xywh,
                        seg_mask=mask.flatten().astype(int).tolist(),
                        detection_score=float(result["scores"][det]),
                        seg_mask_score=(
                            float(result["mask_scores"][det])
                            if no_of_detections > 1
                            else float(result["mask_scores"])
                        ),  # If only one detection, it returns a 0D array
                        point=detection_point,
                        name=result["labels"][det].lower(),
                    )
                )

        response.detections = response_results

        return response


def main():
    """Main entry point for the LangSAM service."""
    rclpy.init()

    # Create service instance
    lang_sam_service = LangSamService()

    lang_sam_service.get_logger().info("LangSAM service initialized successfully")
    rclpy.spin(lang_sam_service)

    lang_sam_service.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()