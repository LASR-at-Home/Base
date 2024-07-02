from deepface import DeepFace

import cv2
import cv2_img
import rospkg
import rospy
import os
import numpy as np
import pandas as pd

from lasr_vision_msgs.msg import Detection
from lasr_vision_msgs.srv import (
    RecogniseRequest,
    RecogniseResponse,
    DetectFacesRequest,
    DetectFacesResponse,
)

from sensor_msgs.msg import Image

from typing import Union, List

DATASET_ROOT = os.path.join(
    rospkg.RosPack().get_path("lasr_vision_deepface"), "datasets"
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
            cv_im,
            detector_backend="opencv",
            enforce_detection=True,
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
    dataset: str,
    name: str,
    images: List[Image],
    debug_publisher: rospy.Publisher,
) -> None:
    dataset_path = os.path.join(DATASET_ROOT, dataset, name)
    if not os.path.exists(dataset_path):
        os.makedirs(dataset_path)
    rospy.loginfo(
        f"Received {len(images)} pictures of {name} and saving to {dataset_path}"
    )
    cv_images: List[Mat] = [cv2_img.msg_to_cv2_img(img) for img in images]
    for i, cv_im in enumerate(cv_images):
        face_cropped_cv_im = _extract_face(cv_im)
        if face_cropped_cv_im is None:
            continue
        cv2.imwrite(os.path.join(dataset_path, f"{name}_{i + 1}.png"), cv_im)
    debug_publisher.publish(cv2_img.cv2_img_to_msg(create_image_collage(cv_images)))

    # Force retraining
    DeepFace.find(
        cv_im,
        os.path.join(DATASET_ROOT, dataset),
        enforce_detection=False,
        silent=True,
        detector_backend="opencv",
    )


def recognise(
    request: RecogniseRequest,
    debug_publisher: rospy.Publisher,
    debug_inference_pub: rospy.Publisher,
    cropped_detect_pub: rospy.Publisher,
) -> RecogniseResponse:
    # Decode the image
    rospy.loginfo("Decoding")
    cv_im = cv2_img.msg_to_cv2_img(request.image_raw)

    response = RecogniseResponse()

    # Run inference
    rospy.loginfo("Running inference")

    try:
        result = DeepFace.find(
            cv_im,
            os.path.join(DATASET_ROOT, request.dataset),
            enforce_detection=True,
            silent=True,
            detector_backend="opencv",
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

        cropped_image = cv_im[:][y : y + h, x : x + w]

        cropped_detect_pub.publish(cv2_img.cv2_img_to_msg(cropped_image))

        # Draw bounding boxes and labels for debugging
        cv2.rectangle(cv_im, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(
            cv_im,
            f"{detection.name} Distance: ({detection.confidence:.2f})",
            (x, y - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )

    # publish to debug topic
    debug_publisher.publish(cv2_img.cv2_img_to_msg(cv_im))
    result = pd.concat(result)
    # check for empty result
    if not result.empty:
        result_paths = list(result["identity"])
        if len(result_paths) > 5:
            result_paths = result_paths[:5]
        result_images = [cv2.imread(path) for path in result_paths]
        debug_inference_pub.publish(
            cv2_img.cv2_img_to_msg(create_image_collage(result_images))
        )

    return response


def detect_faces(
    request: DetectFacesRequest,
    debug_publisher: rospy.Publisher,
) -> DetectFacesResponse:
    cv_im = cv2_img.msg_to_cv2_img(request.image_raw)

    response = DetectFacesResponse()

    try:
        faces = DeepFace.extract_faces(
            cv_im, detector_backend="opencv", enforce_detection=True
        )
    except ValueError:
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

        # Draw bounding boxes and labels for debugging
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

    # publish to debug topic
    debug_publisher.publish(cv2_img.cv2_img_to_msg(cv_im))

    return response
