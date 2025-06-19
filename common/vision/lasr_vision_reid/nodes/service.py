from typing import Dict, Tuple

import torch

import rospy
import torchreid
import torch
from torchvision import transforms
from PIL import Image as PILImage
import os
import rospkg
import numpy as np
from sklearn.metrics.pairwise import cosine_similarity
import cv2
from cv_bridge import CvBridge
import tf2_ros as tf
from facenet_pytorch import MTCNN

from lasr_vision_msgs.srv import (
    Recognise3D,
    Recognise3DRequest,
    Recognise3DResponse,
)
from lasr_vision_msgs.msg import Detection3D

from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from tf2_geometry_msgs import do_transform_point

Mat = np.ndarray


class ReID:

    _dataset: str
    _dataset_root: str
    _model: torch.nn.Module
    _device: torch.device
    _transform: transforms.Compose
    _face_detector: MTCNN
    _db: Dict[str, torch.Tensor]
    _bridge: CvBridge
    _tf_buffer: tf.Buffer
    _tf_listener: tf.TransformListener
    _image_publisher: rospy.Publisher
    _marker_publisher: rospy.Publisher
    _recognise_service: rospy.Service

    def __init__(self, dataset: str):
        self._dataset = dataset
        self._dataset_root = os.path.join(
            rospkg.RosPack().get_path("lasr_vision_reid"), "datasets", self._dataset
        )
        self._model = torchreid.models.build_model(
            "osnet_x1_0", pretrained=True, num_classes=1000
        )
        self._device = (
            torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
        )
        self._model.to(self._device)
        self._model.eval()
        self._transform = transforms.Compose(
            [
                transforms.Resize((256, 128)),
                transforms.ToTensor(),
                transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ]
        )
        self._face_detector = MTCNN(keep_all=True, device=self._device)

        self._db = {}
        self._load_dataset()

        self._bridge = CvBridge()
        self._tf_buffer = tf.Buffer(cache_time=rospy.Duration(10))
        self._tf_listener = tf.TransformListener(self._tf_buffer)

        self._image_publisher = rospy.Publisher(
            "/lasr_vision_reid/recognise/detections", Image, queue_size=10
        )
        self._marker_publisher = rospy.Publisher(
            "/lasr_vision_reid/recognise/points", Marker, queue_size=10
        )

        self._recognise_service = rospy.Service(
            "/lasr_vision_reid/recognise", Recognise3D, self._recognise
        )

    def _load_dataset(self) -> None:
        """
        Load all labeled face images from the dataset directory and extract features in batches.
        """
        if not os.path.exists(self._dataset_root):
            rospy.logerr(f"Dataset path not found: {self._dataset_root}")
            return

        rospy.loginfo(f"Loading dataset from: {self._dataset_root}")

        for label in os.listdir(self._dataset_root):
            label_path = os.path.join(self._dataset_root, label)
            if not os.path.isdir(label_path):
                continue

            image_tensors = []
            for fname in os.listdir(label_path):
                if not fname.lower().endswith(".png"):
                    continue

                img_path = os.path.join(label_path, fname)
                try:
                    pil_im = PILImage.open(img_path).convert("RGB")
                    x = self._transform(pil_im)
                    image_tensors.append(x)
                except Exception as e:
                    rospy.logwarn(f"Failed to process {img_path}: {e}")

            if not image_tensors:
                continue

            batch = torch.stack(image_tensors).to(self._device)
            with torch.no_grad():
                features = self._model(batch).cpu()

            if features.size(0) > 0:
                self._db[label] = features

        rospy.loginfo(f"Loaded {len(self._db)} identities.")

    def _forward_model(self, cv_im: Mat) -> torch.Tensor:
        """
        Perform a forward pass of the model using the input image.
        """
        pil_im = PILImage.fromarray(cv_im)
        x = self._transform(pil_im).unsqueeze(0).to(self._device)
        with torch.no_grad():
            feat = self._model(x).cpu()
        return feat

    def _perform_reid(self, cv_im: Mat) -> Tuple[str, float]:
        """
        Perform person re-identification against the database.
        """
        if not self._db:
            rospy.logwarn("Database is empty. Did you call _load_dataset()?")
            return "unknown", 0.0

        pil_im = PILImage.fromarray(cv_im).convert("RGB")
        x = self._transform(pil_im).unsqueeze(0).to(self._device)

        with torch.no_grad():
            query_feat = self._model(x).cpu().numpy()

        best_match = "unknown"
        best_score = -1.0

        for label, gallery_feats in self._db.items():
            gallery_np = gallery_feats.numpy()
            sim = cosine_similarity(query_feat, gallery_np).mean()

            if sim > best_score:
                best_score = sim
                best_match = label

        return (best_match, float(best_score))

    def _recognise(self, request: Recognise3DRequest) -> Recognise3DResponse:
        response = Recognise3DResponse()

        try:
            cv_im = self._bridge.imgmsg_to_cv2(
                request.image_raw, desired_encoding="bgr8"
            )
            depth_im = self._bridge.imgmsg_to_cv2(
                request.depth_image, desired_encoding="passthrough"
            )
        except Exception as e:
            rospy.logerr(f"Failed to convert image: {e}")
            return response

        h, w, _ = cv_im.shape
        target_frame = request.target_frame or request.depth_image.header.frame_id
        K = request.depth_camera_info.K
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]

        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                request.depth_image.header.frame_id,
                request.depth_image.header.stamp,
                rospy.Duration(1.0),
            )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as e:
            raise rospy.ServiceException(str(e))

        boxes, _ = self._face_detector.detect(cv_im)

        if boxes is None:
            rospy.loginfo("No faces detected.")
            return response

        for box in boxes:
            x1, y1, x2, y2 = [int(v) for v in box]

            x1, y1, x2, y2 = max(0, x1), max(0, y1), min(w, x2), min(h, y2)
            if x1 >= x2 or y1 >= y2:
                continue

            face_crop = cv_im[y1:y2, x1:x2]
            if face_crop.size == 0:
                continue

            label, score = self._perform_reid(face_crop)

            if score < request.threshold:
                continue

            detection = Detection3D()
            detection.name = label
            detection.confidence = score
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

            point = Point(x, y, z)
            point_stamped = PointStamped()
            point_stamped.header = request.depth_image.header
            point_stamped.point = point

            try:
                point_transformed = do_transform_point(point_stamped, transform)
                detection.point = point_transformed.point
            except Exception as e:
                rospy.logwarn(f"Failed to transform point: {e}")
                continue

            response.detections.append(detection)

        self._publish_results(response, cv_im, target_frame)

        return response

    def _publish_results(
        self, response: Recognise3DResponse, cv_im: Mat, frame_id: str
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
            self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        )

        for i, detection in enumerate(response.detections):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = rospy.Time.now()
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


if __name__ == "__main__":
    rospy.init_node("lasr_vision_reid")
    dataset = rospy.get_param("lasr_vision_reid/dataset")
    reid = ReID(dataset)
    rospy.spin()
