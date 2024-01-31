from deepface import DeepFace

import cv2
import cv2_img
import rospkg
import rospy
import os

from lasr_vision_msgs.msg import Detection
from lasr_vision_msgs.srv import RecogniseRequest, RecogniseResponse

from sensor_msgs.msg import Image

DATASET_ROOT = os.path.join(
    rospkg.RosPack().get_path("lasr_vision_deepface"), "datasets"
)


Mat = int  # np.typing.NDArray[np.uint8]


def detect_face(cv_im: Mat) -> Mat | None:
    try:
        faces = DeepFace.extract_faces(
            cv_im,
            target_size=(224, 244),
            detector_backend="mtcnn",
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


def create_dataset(topic: str, dataset: str, name: str, size=50) -> None:
    dataset_path = os.path.join(DATASET_ROOT, dataset, name)
    if not os.path.exists(dataset_path):
        os.makedirs(dataset_path)
    rospy.loginfo(f"Taking {size} pictures of {name} and saving to {dataset_path}")

    for i in range(size):
        img_msg = rospy.wait_for_message(topic, Image)
        cv_im = cv2_img.msg_to_cv2_img(img_msg)
        face_cropped_cv_im = detect_face(cv_im)
        if face_cropped_cv_im is None:
            continue
        cv2.imwrite(os.path.join(dataset_path, f"{name}_{i+1}.png"), face_cropped_cv_im)  # type: ignore
        rospy.loginfo(f"Took picture {i+1}")


def detect(
    request: RecogniseRequest, debug_publisher: rospy.Publisher | None
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
            detector_backend="mtcnn",
            threshold=request.confidence,
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
        detection.confidence = 1.0 - row["distance"][0]
        response.detections.append(detection)

        # Draw bounding boxes and labels for debugging
        cv2.rectangle(cv_im, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(
            cv_im,
            f"{detection.name} ({detection.confidence})",
            (x, y - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )

    # publish to debug topic
    if debug_publisher is not None:
        debug_publisher.publish(cv2_img.cv2_img_to_msg(cv_im))

    return response
