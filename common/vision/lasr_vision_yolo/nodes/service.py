#!/usr/bin/env python3

from typing import Dict, Union, List, Tuple

import rospy

import ultralytics
import torch
import numpy as np
import cv2

from lasr_vision_msgs.srv import (
    YoloDetection,
    YoloDetectionRequest,
    YoloDetectionResponse,
    YoloDetection3D,
    YoloDetection3DRequest,
    YoloDetection3DResponse,
    YoloPoseDetection,
    YoloPoseDetectionRequest,
    YoloPoseDetectionResponse,
    YoloPoseDetection3D,
    YoloPoseDetection3DRequest,
    YoloPoseDetection3DResponse,
)
from lasr_vision_msgs.msg import (
    Detection,
    Detection3D,
    Keypoint,
    Keypoint3D,
    KeypointList,
    Keypoint3DList,
)

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


KEYPOINT_MAPPING: Dict[int, str] = {
    0: "nose",
    1: "left_eye",
    2: "right_eye",
    3: "left_ear",
    4: "right_ear",
    5: "left_shoulder",
    6: "right_shoulder",
    7: "left_elbow",
    8: "right_elbow",
    9: "left_wrist",
    10: "right_wrist",
    11: "left_hip",
    12: "right_hip",
    13: "left_knee",
    14: "right_knee",
    15: "left_ankle",
    16: "right_ankle",
}

SKELETON_CONNECTIONS: List[Tuple[int, int]] = [
    (5, 6),  # shoulders
    (5, 7),
    (7, 9),  # left arm
    (6, 8),
    (8, 10),  # right arm
    (11, 12),  # hips
    (5, 11),
    (6, 12),  # torso
    (11, 13),
    (13, 15),  # left leg
    (12, 14),
    (14, 16),  # right leg
    (0, 1),
    (0, 2),  # nose to eyes
    (1, 3),
    (2, 4),  # eyes to ears
    (0, 5),
    (0, 6),  # nose to shoulders
]


class YOLOService:

    _cache: Dict[str, ultralytics.YOLO]
    _image_publishers: Dict[str, rospy.Publisher]
    _marker_publishers: Dict[str, rospy.Publisher]
    _bridge: CvBridge

    def __init__(self):

        self._cache = {}
        self._device = rospy.get_param(
            "~device", "cuda:0" if torch.cuda.is_available() else "cpu"
        )

        for model in rospy.get_param("~preload", []):
            self._maybe_load_model(model)

        self._image_publishers = {}
        self._marker_publishers = {}
        self._bridge = CvBridge()

        rospy.Service("/yolo/detect", YoloDetection, self._detect)
        rospy.Service("/yolo/detect3d", YoloDetection3D, self._detect3d)
        rospy.Service("/yolo/detect_pose", YoloPoseDetection, self._detect_keypoints)
        rospy.Service(
            "/yolo/detect3d_pose", YoloPoseDetection3D, self._detect_keypoints3d
        )

    def _detect(self, req: YoloDetectionRequest) -> YoloDetectionResponse:
        response = YoloDetectionResponse()

        cv_im = self._bridge.imgmsg_to_cv2(req.image_raw, desired_encoding="bgr8")
        results = self._yolo(cv_im, req.model, req.confidence)

        has_masks = results.masks is not None

        for result in results:

            detection = Detection()
            detection.name = result.names[result.boxes.cls.int().item()]
            detection.confidence = result.boxes.conf.item()
            detection.xywh = (
                result.boxes.xywh.round().int().squeeze().cpu().numpy().tolist()
            )

            if has_masks:
                detection.xyseg = (
                    np.array(result.masks.xy).flatten().round().astype(int).tolist()
                )

            response.detected_objects.append(detection)

        self._publish_results(req, results, response)

        return response

    def _detect3d(self, req: YoloDetection3DRequest) -> YoloDetection3DResponse:
        response = YoloDetection3DResponse()

        cv_im = self._bridge.imgmsg_to_cv2(req.image_raw, desired_encoding="bgr8")
        results = self._yolo(cv_im, req.model, req.confidence)
        depth_im = self._bridge.imgmsg_to_cv2(
            req.depth_image, desired_encoding="passthrough"
        )
        K = req.depth_camera_info.K
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]

        results = self._yolo(cv_im, req.model, req.confidence)

        for result in results:
            detection = Detection3D()
            detection.name = result.names[result.boxes.cls.int().item()]
            detection.confidence = result.boxes.conf.item()
            x, y, w, h = (
                result.boxes.xywh.round().int().squeeze().cpu().numpy().tolist()
            )
            detection.xywh = [x, y, w, h]

            has_mask = result.masks is not None
            if has_mask:
                detection.xyseg = (
                    np.array(result.masks.xy).flatten().round().astype(int).tolist()
                )
                contours = np.array(detection.xyseg).reshape(-1, 2)
                mask = np.zeros(depth_im.shape[:2], dtype=np.uint8)
                cv2.fillPoly(mask, [contours], color=255)
                roi = cv2.bitwise_and(depth_im, depth_im, mask=mask)
                v, u = np.where(roi)
                z = depth_im[v, u]
                valid = z > 0
                z = z[valid]
                u = u[valid]
                v = v[valid]
                x = z * (u - cx) / fx
                y = z * (v - cy) / fy
                points = np.stack((x, y, z), axis=1)
                x, y, z = np.median(points, axis=0)
                detection.point.x = x
                detection.point.y = y
                detection.point.z = z
            else:
                rospy.logwarn(
                    "3D Estimation is not implemented when masks aren't available."
                )

            response.detected_objects.append(detection)

        self._publish_results(req, results, response)

        return response

    def _detect_keypoints(
        self, req: YoloPoseDetectionRequest
    ) -> YoloPoseDetectionResponse:
        response = YoloPoseDetectionResponse()

        cv_im = self._bridge.imgmsg_to_cv2(req.image_raw, desired_encoding="bgr8")
        results = self._yolo(cv_im, req.model, req.confidence)

        for result in results:
            keypoints = KeypointList()
            for idx, name in KEYPOINT_MAPPING.items():
                x = result.keypoints.xy.squeeze()[idx, 0].round().int().item()
                y = result.keypoints.xy.squeeze()[idx, 1].round().int().item()
                conf = result.keypoints.conf.squeeze()[idx].item()
                if conf > 0.0:
                    keypoints.keypoints.append(Keypoint(name, x, y))
            response.detections.append(keypoints)

        self._publish_results(req, results, response)

        return response

    def _detect_keypoints3d(
        self, req: YoloPoseDetection3DRequest
    ) -> YoloPoseDetection3DResponse:
        response = YoloPoseDetection3DResponse()

        cv_im = self._bridge.imgmsg_to_cv2(req.image_raw, desired_encoding="bgr8")
        results = self._yolo(cv_im, req.model, req.confidence)
        depth_im = self._bridge.imgmsg_to_cv2(
            req.depth_image, desired_encoding="passthrough"
        )
        K = req.depth_camera_info.K
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]

        results = self._yolo(cv_im, req.model, req.confidence)

        for result in results:
            keypoints = Keypoint3DList()
            for idx, name in KEYPOINT_MAPPING.items():
                u = result.keypoints.xy.squeeze()[idx, 0].round().int().item()
                v = result.keypoints.xy.squeeze()[idx, 1].round().int().item()

                # TODO check u,v are in range.

                conf = result.keypoints.conf.squeeze()[idx].item()
                if conf > 0.0:
                    z = depth_im[v, u]
                    x = z * (u - cx) / fx
                    y = z * (v - cy) / fy
                    if np.isnan(x) or np.isnan(y) or np.isnan(z):
                        continue
                    keypoints.keypoints.append(Keypoint3D(name, Point(x, y, z)))
            response.detections.append(keypoints)

        self._publish_results(req, results, response)

        return response

    def _maybe_load_model(self, model_name: str) -> ultralytics.YOLO:
        if model_name in self._cache:
            return self._cache[model_name]

        model = self._cache[model_name] = ultralytics.YOLO(model_name).to(self._device)

        rospy.loginfo(f"Loaded {model_name} model on {self._device}")
        return model

    def _publish_results(
        self,
        req: Union[YoloDetectionRequest, YoloDetection3DRequest],
        results: ultralytics.engine.results.Results,
        response: Union[YoloDetectionResponse, YoloDetection3DResponse],
    ) -> None:

        if req.model in self._image_publishers:
            image_publisher = self._image_publishers[req.model]
        else:
            image_publisher = self._image_publishers[req.model] = rospy.Publisher(
                f"/yolo/detect/{req.model}".replace("-", "_").replace(".", "_"),
                Image,
                queue_size=10,
            )

        image_publisher.publish(
            self._bridge.cv2_to_imgmsg(results.plot(), encoding="bgr8")
        )

        if isinstance(response, YoloDetection3DResponse):

            if req.model in self._marker_publishers:
                marker_publisher = self._marker_publishers[req.model]
            else:
                marker_publisher = self._marker_publishers[req.model] = rospy.Publisher(
                    f"/yolo/detect3d/{req.model}".replace("-", "_").replace(".", "_"),
                    Marker,
                    queue_size=10,
                )

            for i, detection in enumerate(response.detected_objects):

                marker = Marker()
                marker.header.frame_id = req.depth_image.header.frame_id
                marker.header.stamp = rospy.Time.now()
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose.position = detection.point

                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1

                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0

                marker_publisher.publish(marker)

        elif isinstance(response, YoloPoseDetection3DResponse):
            if req.model in self._marker_publishers:
                marker_publisher = self._marker_publishers[req.model]
            else:
                marker_publisher = self._marker_publishers[req.model] = rospy.Publisher(
                    f"/yolo/pose3d/{req.model}".replace("-", "_").replace(".", "_"),
                    MarkerArray,
                    queue_size=10,
                )

            marker_array = MarkerArray()
            marker_id = 0

            for detection_idx, keypoint_list in enumerate(response.detections):
                points = [kp.point for kp in keypoint_list.keypoints]

                # Add spheres for each keypoint
                for i, pt in enumerate(points):
                    marker = Marker()
                    marker.header.frame_id = req.depth_image.header.frame_id
                    marker.header.stamp = rospy.Time.now()
                    marker.id = marker_id
                    marker.ns = f"person_{detection_idx}"
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    marker.pose.position = pt
                    marker.scale.x = 0.05
                    marker.scale.y = 0.05
                    marker.scale.z = 0.05
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 1.0
                    marker_array.markers.append(marker)
                    marker_id += 1

                # Add line segments to represent skeleton
                for a, b in SKELETON_CONNECTIONS:
                    if a < len(points) and b < len(points):
                        line_marker = Marker()
                        line_marker.header.frame_id = req.depth_image.header.frame_id
                        line_marker.header.stamp = rospy.Time.now()
                        line_marker.id = marker_id
                        line_marker.ns = f"person_{detection_idx}_lines"
                        line_marker.type = Marker.LINE_LIST
                        line_marker.action = Marker.ADD
                        line_marker.scale.x = 0.02
                        line_marker.color.r = 0.0
                        line_marker.color.g = 0.0
                        line_marker.color.b = 1.0
                        line_marker.color.a = 1.0
                        line_marker.points.append(points[a])
                        line_marker.points.append(points[b])
                        marker_array.markers.append(line_marker)
                        marker_id += 1

            marker_publisher.publish(marker_array)

    def _yolo(self, img, model: str, conf: float) -> ultralytics.engine.results.Results:
        yolo = self._maybe_load_model(model)
        results = yolo(img, conf=conf)[0]
        return results


if __name__ == "__main__":
    rospy.init_node("yolo")
    yolo = YOLOService()
    rospy.spin()
