"""
how to run reid:
1) run the simulation, then run this service node:
    ros2 run lasr_vision_reid service
2) In a separate terminal, run the add_face node to detect faces:
    ros2 run lasr_vision_reid add_face
3) Then in another terminal run the relay node to display detections to the lasr_vision_reid topic:
    ros2 run lasr_vision_reid relay_3d
"""

from typing import Dict, Tuple, Optional, List
import os

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import numpy as np
from sklearn.metrics.pairwise import cosine_similarity
import cv2
from cv_bridge import CvBridge
import tf2_ros as tf
from deepface import DeepFace

from lasr_vision_interfaces.srv import Recognise3D, AddFace
from lasr_vision_interfaces.msg import Detection3D

from tensorflow.compat.v1 import ConfigProto
from tensorflow.compat.v1 import InteractiveSession

from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from tf2_geometry_msgs import do_transform_point

Mat = np.ndarray


class ReID(Node):

    _db: Dict[str, List[np.ndarray]]
    _bridge: CvBridge
    _tf_buffer: tf.Buffer
    _tf_listener: tf.TransformListener
    _tf_client: rclpy.client.Client
    _image_publisher: rclpy.publisher.Publisher
    _marker_publisher: rclpy.publisher.Publisher
    _recognise_3d_service: rclpy.service.Service
    _recognise_2d_service: rclpy.service.Service
    _add_face_service: rclpy.service.Service

    def __init__(self):
        super().__init__("lasr_vision_reid_service")
        self._db = {}

        self._bridge = CvBridge()
        self._tf_buffer = tf.Buffer(cache_time=Duration(seconds=10))
        self._tf_listener = tf.TransformListener(self._tf_buffer, self)

        self._image_publisher = self.create_publisher(
            Image, "/lasr_vision_reid/recognise/detections", 10
        )
        self._marker_publisher = self.create_publisher(
            Marker, "/lasr_vision_reid/recognise/points", 10
        )

        self._recognise_service = self.create_service(
            Recognise3D, "/lasr_vision_reid/recognise", self._recognise
        )

        self._add_face_service = self.create_service(
            AddFace, "/lasr_vision_reid/add_face", self._add_face
        )
        _ = DeepFace.build_model("VGG-Face")

    def _extract_embeddings(self, im: np.ndarray) -> List[np.ndarray]:
        """
        Use DeepFace to extract an embedding of a face.
        """
        results = DeepFace.represent(
            img_path=im, model_name="VGG-Face", enforce_detection=False
        )
        embeddings = [np.array(entry["embedding"]) for entry in results]
        return embeddings

    def _recognise(
        self, request: Recognise3D.Request, response: Recognise3D.Response
    ) -> Recognise3D.Response:
        response.detections = []

        try:
            cv_im = self._bridge.imgmsg_to_cv2(
                request.image_raw, desired_encoding="rgb8"
            )
            depth_im = self._bridge.imgmsg_to_cv2(
                request.depth_image, desired_encoding="passthrough"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return response

        h, w, _ = cv_im.shape
        target_frame = request.target_frame or request.depth_image.header.frame_id
        K = request.depth_camera_info.k
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]

        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                request.depth_image.header.frame_id,
                request.depth_image.header.stamp,
                rclpy.duration.Duration(seconds=2.0),  # Longer timeout for async
            )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as e:
            self.get_logger().error(
                f"Failed to find transform between {request.depth_image.header.frame_id}, and the target frame {target_frame}"
            )
            return response

        try:
            # Get face embeddings and bounding boxes from DeepFace (face detection + embedding)
            results = DeepFace.represent(
                img_path=cv_im,
                model_name="VGG-Face",
                enforce_detection=False,
                detector_backend="opencv",
                align=True,
                max_faces=None,
            )
        except Exception as e:
            self.get_logger().warning(f"DeepFace representation failed: {e}")
            return response

        if not results:
            self.get_logger().info("No faces detected.", once=True)
            return response

        for face_data in results:
            embedding = np.array(face_data["embedding"])
            region = face_data["facial_area"]
            x1, y1 = region["x"], region["y"]
            x2, y2 = x1 + region["w"], y1 + region["h"]

            # Clamp bounding box within image
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)

            # Compare embedding with database entries
            best_label = "unknown"
            best_score = -1.0
            for label, embeddings in self._db.items():
                sims = [
                    cosine_similarity([embedding], [db_emb])[0][0]
                    for db_emb in embeddings
                ]
                avg_sim = np.mean(sims)
                if avg_sim > best_score:
                    best_score = avg_sim
                    best_label = label

            if best_score < request.threshold:
                continue

            detection = Detection3D()
            detection.name = best_label
            detection.confidence = best_score
            detection.xywh = [x1, y1, x2 - x1, y2 - y1]

            roi_depth = depth_im[y1:y2, x1:x2]
            v, u = np.where(roi_depth > 0)
            if len(u) == 0:
                continue

            z = roi_depth[v, u]
            u = u + x1
            v = v + y1

            x = z * (u - cx) / fx
            y = z * (v - cy) / fy
            points = np.stack((x, y, z), axis=1)
            x, y, z = np.median(points, axis=0)

            point = Point()
            point.x = x / 1000
            point.y = y / 1000
            point.z = z / 1000
            point_stamped = PointStamped()
            point_stamped.header = request.depth_image.header
            point_stamped.point = point

            try:
                point_transformed = do_transform_point(point_stamped, transform)
                detection.point = point_transformed.point
            except Exception as e:
                self.get_logger().warning
                (f"Point transformation failed: {e}.")
                continue

            response.detections.append(detection)

        self._publish_results(response, cv_im, target_frame)
        return response

    def _add_face(
        self, request: AddFace.Request, response: AddFace.Response
    ) -> AddFace.Response:
        response.success = False

        try:
            # Convert ROS Image message to OpenCV image (BGR)
            cv_im = self._bridge.imgmsg_to_cv2(
                request.image_raw, desired_encoding="rgb8"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return response

        try:
            # Use DeepFace to get embeddings, assume single face
            results = DeepFace.represent(
                img_path=cv_im,
                model_name="VGG-Face",
                enforce_detection=False,  # allow detection attempts even if uncertain
                detector_backend="opencv",
                align=True,
                max_faces=1,
            )
        except Exception as e:
            self.get_logger().warning(f"Failed to extract embedding: {e}")
            return response

        if not results:
            self.get_logger().warning("No face detected in the image.")
            return response

        embedding = np.array(results[0]["embedding"])
        name = request.name.strip()

        if not name:
            self.get_logger().warning("Empty name provided for face addition.")
            return response

        # Add embedding to database
        if name not in self._db:
            self._db[name] = []

        self._db[name].append(embedding)
        response.success = True
        self.get_logger().info(
            f"Added face embedding for {name}, total samples: {len(self._db[name])}"
        )

        return response

    def _publish_results(
        self, response: Recognise3D.Response, cv_im: Mat, frame_id: str
    ) -> None:

        annotated = cv_im.copy()
        for detection in response.detections:
            x, y, w, h = detection.xywh
            cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
            label = f"{detection.name} ({detection.confidence:.2f})"
            cv2.putText(
                annotated,
                label,
                (x, y - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

        self._image_publisher.publish(
            self._bridge.cv2_to_imgmsg(annotated, encoding="rgb8")
        )

        for i, detection in enumerate(response.detections):
            if detection.name == "guest1" or detection.name == "guest2":
                marker = Marker()
                marker.header.frame_id = frame_id
                marker.header.stamp = (
                    self.get_clock().now().to_msg()
                )  # Convert to message type
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position = detection.point
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.r = 0.0
                marker.color.g = 0.5
                marker.color.b = 1.0
                marker.color.a = 1.0

                self._marker_publisher.publish(marker)


def main():
    rclpy.init()

    config = ConfigProto()
    config.gpu_options.per_process_gpu_memory_fraction = 0.20
    session = InteractiveSession(config=config)

    reid = ReID()
    reid.get_logger().info("Vision reid service is ready!", once=True)

    rclpy.spin(reid)
    reid.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
