from __future__ import annotations
from typing import List
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import re
import tensorflow as tf
import cv2_img
from sensor_msgs.msg import Image as SensorImage
from tf_bodypix.api import download_model, load_model, BodyPixModelPaths
from lasr_vision_msgs.msg import BodyPixMask, BodyPixKeypoint, BodyPixKeypointNormalized
from lasr_vision_msgs.srv import (
    BodyPixMaskDetection,
    BodyPixKeypointDetection,

)

BodyPixKeypointDetection_Request = BodyPixKeypointDetection.Request()
BodyPixKeypointDetection_Response = BodyPixKeypointDetection.Response()
BodyPixMaskDetection_Request = BodyPixMaskDetection.Request()
BodyPixMaskDetection_Response = BodyPixMaskDetection.Response()


# model cache
loaded_models = {}


def snake_to_camel(snake_str):
    components = snake_str.split("_")
    return components[0] + "".join(x.title() for x in components[1:])


def camel_to_snake(name):
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


def load_model_cached(dataset: str):
    """
    Load a model into cache
    """
    model = None
    if dataset in loaded_models:
        model = loaded_models[dataset]
    else:
        if dataset == "resnet50":
            name = download_model(BodyPixModelPaths.RESNET50_FLOAT_STRIDE_16)
            model = load_model(name)
        elif dataset == "mobilenet50":
            name = download_model(BodyPixModelPaths.MOBILENET_FLOAT_50_STRIDE_8)
            model = load_model(name)
        elif dataset == "mobilenet100":
            name = download_model(BodyPixModelPaths.MOBILENET_FLOAT_100_STRIDE_8)
            model = load_model(name)
        else:
            model = load_model(dataset)
        loaded_models[dataset] = model
    return model


class BodyPixNode(Node):
    def __init__(self):
        super().__init__('bodypix_service_node')
        self.mask_service = self.create_service(
            BodyPixMaskDetection, 'detect_masks', self.detect_masks
        )
        self.keypoint_service = self.create_service(
            BodyPixKeypointDetection, 'detect_keypoints', self.detect_keypoints
        )
        self.debug_publisher_mask = self.create_publisher(
            SensorImage, '/bodypix/mask_debug', 1
        )
        self.debug_publisher_keypoint = self.create_publisher(
            SensorImage, '/bodypix/keypoint_debug', 1
        )

    def run_inference(self, dataset: str, confidence: float, img: SensorImage):
        self.get_logger().info("Decoding")
        img = cv2_img.msg_to_cv2_img(img)

        self.get_logger().info("Loading model")
        model = load_model_cached(dataset)

        self.get_logger().info("Running inference")
        result = model.predict_single(img)

        mask = result.get_mask(threshold=confidence)
        self.get_logger().info("Inference complete")
        return result, mask

    def detect_masks(
        self, request: BodyPixMaskDetection_Request, response: BodyPixMaskDetection_Response
    ) -> BodyPixMaskDetection_Response:
        result, mask = self.run_inference(request.dataset, request.confidence, request.image_raw)

        masks = []
        for part_name in request.parts:
            part_mask = result.get_part_mask(mask=tf.identity(mask), part_names=[part_name]).squeeze()

            if np.max(part_mask) == 0:
                self.get_logger().warn(f"No masks found for part {part_name}")
                continue

            bodypix_mask = BodyPixMask()
            bodypix_mask.mask = part_mask.flatten().astype(bool).tolist()
            bodypix_mask.shape = list(part_mask.shape)
            bodypix_mask.part_name = part_name
            masks.append(bodypix_mask)

        # Publish to debug topic
        if self.debug_publisher_mask is not None:
            from tf_bodypix.draw import draw_poses
            coloured_mask = result.get_colored_part_mask(mask).astype(np.uint8)
            poses = result.get_poses()
            coloured_mask = draw_poses(
                coloured_mask.copy(),
                poses,
                keypoints_color=(255, 100, 100),
                skeleton_color=(100, 100, 255),
            )
            self.debug_publisher_mask.publish(cv2_img.cv2_img_to_msg(coloured_mask))

        response.masks = masks
        return response

    def detect_keypoints(
        self, request: BodyPixKeypointDetection_Request, response: BodyPixKeypointDetection_Response
    ) -> BodyPixKeypointDetection_Response:
        result, mask = self.run_inference(request.dataset, request.confidence, request.image_raw)

        poses = result.get_poses()
        detected_keypoints: List[BodyPixKeypoint] = []
        detected_keypoints_normalized: List[BodyPixKeypointNormalized] = []

        for pose in poses:
            for keypoint in pose.keypoints.values():
                x = int(keypoint.position.x)
                y = int(keypoint.position.y)
                try:
                    if not request.keep_out_of_bounds:
                        if x < 0.0 or y < 0.0:
                            continue
                        if x >= mask.shape[1] or y >= mask.shape[0]:
                            continue
                except:
                    continue
                self.get_logger().info(f"Keypoint {keypoint.part} at {x}, {y} is in mask")
                detected_keypoints.append(
                    BodyPixKeypoint(keypoint_name=keypoint.part, x=x, y=y)
                )
                detected_keypoints_normalized.append(
                    BodyPixKeypointNormalized(
                        keypoint_name=keypoint.part,
                        x=float(x) / mask.shape[1],
                        y=float(y) / mask.shape[0],
                    )
                )

        if self.debug_publisher_keypoint is not None:
            from tf_bodypix.draw import draw_poses
            coloured_mask = result.get_colored_part_mask(mask).astype(np.uint8)
            coloured_mask = draw_poses(
                coloured_mask.copy(),
                poses,
                keypoints_color=(255, 100, 100),
                skeleton_color=(100, 100, 255),
            )
            for keypoint in detected_keypoints:
                cv2.putText(
                    coloured_mask,
                    f"{keypoint.keypoint_name}",
                    (int(keypoint.x), int(keypoint.y)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
            self.debug_publisher_keypoint.publish(cv2_img.cv2_img_to_msg(coloured_mask))

        response.keypoints = detected_keypoints
        response.normalized_keypoints = detected_keypoints_normalized
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BodyPixNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

