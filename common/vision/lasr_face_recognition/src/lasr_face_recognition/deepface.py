from deepface import DeepFace
import cv2
import cv2_img
import rospkg
import rospy
import os

from lasr_vision_msgs.msg import Detection
from lasr_vision_msgs.srv import RecogniseRequest, RecogniseResponse

DATASET_ROOT = os.path.join(rospkg.RosPack().get_path("lasr_face_recognition"), "datasets")

def detect(request : RecogniseRequest, debug_publisher: rospy.Publisher | None) -> RecogniseResponse:
    
    # Decode the image
    rospy.loginfo("Decoding")
    cv_im = cv2_img.msg_to_cv2_img(request.image_raw)

    # Run inference
    rospy.loginfo("Running inference")
    result = DeepFace.find(cv_im, os.path.join(DATASET_ROOT, request.dataset), enforce_detection=False)

    response = RecogniseResponse()

    for row in result:
        if row.empty:
            continue
        detection = Detection()
        detection.name = row["identity"][0].split("/")[-1].split("_")[0]
        x, y, w, h = row["source_x"][0], row["source_y"][0], row["source_w"][0], row["source_h"][0]
        detection.xywh = [x, y, w, h]
        response.detections.append(detection)

        # Draw bounding boxes and labels for debugging
        cv2.rectangle(cv_im, (x, y), (x+w, y+h), (0, 0, 255), 2)
        cv2.putText(cv_im, detection.name, (x,y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # publish to debug topic
    if debug_publisher is not None:
        debug_publisher.publish(cv2_img.cv2_img_to_msg(cv_im))

    return response