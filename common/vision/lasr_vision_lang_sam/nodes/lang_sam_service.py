from typing import List

import os
import rospy
import cv2
import cv2_img
import sys
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

    _model: LangSAM
    _service: rospy.Service
    _tf_service: rospy.ServiceProxy
    debug_publisher: rospy.Publisher

    def __init__(self, use_gpu: bool = True):
        self.debug_publisher = rospy.Publisher(
            "/lasr_vision/lang_sam/debug", SensorImage, queue_size=10
        )

        if not use_gpu:
            os.environ["CUDA_VISIBLE_DEVICES"] = ""

        self._model = LangSAM()
        self._service = rospy.Service("/lasr_vision/lang_sam", LangSam, self._lang_sam)
        self._tf_service = rospy.ServiceProxy(
            "/tf_server/transform_point", TransformPoint
        )
        self._tf_service.wait_for_service()
        rospy.loginfo("/lasr_vision/lang_sam service is ready!")

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

    def _lang_sam(self, request: LangSamRequest) -> LangSamResponse:
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
        K = request.depth_camera_info.K
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
                rospy.logwarn("No detections found.")
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
                point_transformed = tf_response.transformed_point_stamped

                detection_point = point_transformed.point

                response_results.append(
                    LangSamDetection(
                        xywh=xywh,
                        seg_mask=mask.flatten().astype(int).tolist(),
                        detection_score=result["scores"][det],
                        seg_mask_score=(
                            result["mask_scores"][det]
                            if no_of_detections > 1
                            else float(result["mask_scores"])
                        ),  # If only one detection, it returns a 0D array
                        point=detection_point,
                        name=result["labels"][det].lower(),
                    )
                )

        response = LangSamResponse(detections=response_results)

        return response


if __name__ == "__main__":
    rospy.init_node("lasr_vision_lang_sam")
    lang_sam_service = LangSamService()
    rospy.spin()
