from deepface import DeepFace
import cv2
import cv2_img
import rospkg
import rospy
import os
import numpy as np

from lasr_vision_msgs.msg import Detection
from lasr_vision_msgs.srv import RecogniseRequest, RecogniseResponse

DATASET_ROOT = os.path.join(rospkg.RosPack().get_path("lasr_face_recognition"), "datasets")

Mat = int#np.typing.NDArray[np.uint8]

def detect_face(cv_im : Mat) -> Mat | None:
    faces = DeepFace.extract_faces(cv_im, target_size=(224, 244), detector_backend="mtcnn", enforce_detection = False)
    if not faces:
        return None
    facial_area = faces[0]["facial_area"]
    x,y,w,h = facial_area["x"], facial_area["y"], facial_area["w"], facial_area["h"]
    return cv_im[:][y:y+h, x:x+w]

def detect(request : RecogniseRequest, debug_publisher: rospy.Publisher | None) -> RecogniseResponse:
    
    # Decode the image
    rospy.loginfo("Decoding")
    cv_im = cv2_img.msg_to_cv2_img(request.image_raw)

    # Run inference
    rospy.loginfo("Running inference")
    result = DeepFace.find(cv_im, os.path.join(DATASET_ROOT, request.dataset), enforce_detection=False, silent=True)

    response = RecogniseResponse()

    for row in result:
        if row.empty:
            continue
        detection = Detection()
        detection.name = row["identity"][0].split("/")[-1].split("_")[0]
        x, y, w, h = row["source_x"][0], row["source_y"][0], row["source_w"][0], row["source_h"][0]
        detection.xywh = [x, y, w, h]
        confidence = row["VGG-Face_cosine"]
        response.detections.append(detection)

        # Draw bounding boxes and labels for debugging
        cv2.rectangle(cv_im, (x, y), (x+w, y+h), (0, 0, 255), 2)
        cv2.putText(cv_im, f"{detection.name} ({confidence})", (x,y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # publish to debug topic
    if debug_publisher is not None:
        debug_publisher.publish(cv2_img.cv2_img_to_msg(cv_im))

    return response