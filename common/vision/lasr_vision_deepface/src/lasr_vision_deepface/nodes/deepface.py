from deepface import DeepFace

import cv2
import cv2_img
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np
import pandas as pd

from lasr_vision_interfaces.msg import Detection
from lasr_vision_interfaces.srv import Recognise, DetectFaces

from sensor_msgs.msg import Image

from typing import Union, List

Recognise_Request = Recognise.Request
Recognise_Response = Recognise.Response
DetectFaces_Request = DetectFaces.Request
DetectFaces_Response = DetectFaces.Response


DATASET_ROOT = os.path.join(
    get_package_share_directory("lasr_vision_deepface"), "datasets"
)

Mat = np.ndarray


def create_image_collage(images, output_size=(640, 480)):
    # Calculate grid dimensions
    num_images = len(images)
    rows = int(np.sqrt(num_images))
    print(num_images, rows)
    cols = (num_images + rows - 1) // rows  # Ceiling division

    # Resize images to fit in the grid
    resized_images = [
        cv2.resize(img, (output_size[0] // cols, output_size[1] // rows))
        for img in images
    ]

    # Create the final image grid
    grid_image = np.zeros((output_size[1], output_size[0], 3), dtype=np.uint8)

    # Populate the grid with resized images
    for i in range(rows):
        for j in range(cols):
            idx = i * cols + j
            if idx < num_images:
                y_start = i * (output_size[1] // rows)
                y_end = (i + 1) * (output_size[1] // rows)
                x_start = j * (output_size[0] // cols)
                x_end = (j + 1) * (output_size[0] // cols)

                grid_image[y_start:y_end, x_start:x_end] = resized_images[idx]

    return grid_image


def _extract_face(cv_im: Mat) -> Union[Mat, None]:
    try:
        faces = DeepFace.extract_faces(
            cv_im, detector_backend="mtcnn", enforce_detection=True
        )
    except ValueError:
        return None
    facial_area = faces[0]["facial_area"]
    x, y, w, h = facial_area["x"], facial_area["y"], facial_area["w"], facial_area["h"]

    # add padding to the face
    x -= 10
    y -= 10
    w += 20
    h += 20

    return cv_im[:][y : y + h, x : x + w]


def create_dataset(
    dataset: str, name: str, images: List[Image], debug_publisher=None, logger=None
) -> None:
    dataset_path = os.path.join(DATASET_ROOT, dataset, name)
    if not os.path.exists(dataset_path):
        os.makedirs(dataset_path)
    if logger:
        logger.info(
            f"Received {len(images)} pictures of {name} and saving to {dataset_path}"
        )
    cv_images: List[Mat] = [cv2_img.msg_to_cv2_img(img) for img in images]
    for i, cv_im in enumerate(cv_images):
        face_cropped_cv_im = _extract_face(cv_im)
        if face_cropped_cv_im is None:
            continue
        cv2.imwrite(
            os.path.join(dataset_path, f"{name}_{i + 1}.png"), face_cropped_cv_im
        )
    debug_publisher.publish(cv2_img.cv2_img_to_msg(create_image_collage(cv_images)))

    # Force retraining
    DeepFace.find(
        cv_im,
        os.path.join(DATASET_ROOT, dataset),
        enforce_detection=False,
        silent=True,
        detector_backend="mtcnn",
    )


def recognise(cv_im: Mat, debug_publisher=None, logger=None, cropped_detect_pub=None):
    """Recognises a face from an image (allows direct image input)."""

    response = Recognise_Response()

    # Run inference
    if logger:
        logger.info("Running DeepFace recognition...")

    try:
        result = DeepFace.find(
            cv_im,
            os.path.join(DATASET_ROOT, "your_dataset"),  # Make sure this path exists
            enforce_detection=True,
            silent=True,
            detector_backend="mtcnn",
        )
    except ValueError:
        return response

    for row in result:
        if row.empty:
            continue
        detection = Detection()
        detection.name = row["identity"][0].split("/")[-1].split("_")[0]
        x, y, w, h = (
            row["source_x"][0],
            row["source_y"][0],
            row["source_w"][0],
            row["source_h"][0],
        )
        detection.xywh = [x, y, w, h]
        detection.confidence = row["distance"][0]
        response.detections.append(detection)

        cropped_image = cv_im[y : y + h, x : x + w]
        cropped_detect_pub.publish(cv2_img.cv2_img_to_msg(cropped_image))

        # Draw bounding boxes
        cv2.rectangle(cv_im, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(
            cv_im,
            f"{detection.name} ({detection.confidence:.2f})",
            (x, y - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )

    debug_publisher.publish(cv2_img.cv2_img_to_msg(cv_im))
    return response


def detect_faces(request: DetectFaces_Request, debug_publisher=None, logger=None):
    if logger:
        logger.info("detect_faces service called.")

    if not request.image_raw.encoding:
        request.image_raw.encoding = "bgr8"  # Use "rgb8" if needed

    cv_im = cv2_img.msg_to_cv2_img(request.image_raw)

    if cv_im is None or cv_im.size == 0:
        if logger:
            logger.error("Received an empty or invalid image in detect_faces!")
        return DetectFaces_Response()

    if logger:
        logger.info(f"Processing image: shape={cv_im.shape}")

    response = DetectFaces_Response()

    try:
        faces = DeepFace.extract_faces(
            cv_im, detector_backend="mtcnn", enforce_detection=True
        )
        if logger:
            logger.info(f"DeepFace detected {len(faces)} faces.")
    except ValueError as e:
        if logger:
            logger.error(f"DeepFace error: {e}")
        return response

    for i, face in enumerate(faces):
        detection = Detection()
        detection.name = f"face_{i}"
        x, y, w, h = (
            face["facial_area"]["x"],
            face["facial_area"]["y"],
            face["facial_area"]["w"],
            face["facial_area"]["h"],
        )
        detection.xywh = [x, y, w, h]
        detection.confidence = 1.0
        response.detections.append(detection)

        cv2.rectangle(cv_im, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(
            cv_im,
            f"face_{i}",
            (x, y - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )

    if debug_publisher:
        debug_publisher.publish(cv2_img.cv2_img_to_msg(cv_im))
        if logger:
            logger.info("Published debug image.")

    return response
