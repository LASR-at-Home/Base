#!/usr/bin/env python3
from lasr_vision_msgs.srv import (
    ClipLearnFaceRequest,
    ClipLearnFace,
    ClipLearnFaceResponse,
    CroppedDetection,
    CroppedDetectionRequest,
    CroppedDetectionResponse,
    ClipRecogniseFaceRequest,
    ClipRecogniseFace,
    ClipRecogniseFaceResponse,
)
from lasr_vision_msgs.msg import CDRequest
from sensor_msgs.msg import Image
import cv2
from cv2_img import msg_to_cv2_img, cv2_img_to_msg
import rospy
from typing import List
import numpy as np


if __name__ == "__main__":
    rospy.init_node("clip_encoder_test")
    cropped_detector = rospy.ServiceProxy("/vision/cropped_detection", CroppedDetection)
    learn_face_service = rospy.ServiceProxy("/vision/learn_face", ClipLearnFace)
    detect_face_service = rospy.ServiceProxy(
        "/vision/face_detection", ClipRecogniseFace
    )
    debug_pub = rospy.Publisher("/clip/recognise/debug", Image, queue_size=1)
    input_str = ""
    while True:
        input_str = input("Please enter your name and hit enter to learn your face: ")
        if input_str == "done":
            break
        person_1_imgs = []
        for i in range(10):
            cropped_response = cropped_detector(
                CroppedDetectionRequest(
                    [
                        CDRequest(
                            method="centered",
                            use_mask=True,
                            object_names=["person"],
                            yolo_model="yolo11n-seg.pt",
                            yolo_model_confidence=0.8,
                            yolo_nms_threshold=0.4,
                        )
                    ]
                )
            )
            rospy.sleep(0.1)
            try:
                person_1_imgs.append(cropped_response.responses[0].cropped_imgs[0])
            except:
                continue

        learn_face_service(ClipLearnFaceRequest(raw_imgs=person_1_imgs, name=input_str))

    # Run inference
    while not rospy.is_shutdown():
        cropped_response = cropped_detector(
            CroppedDetectionRequest(
                [
                    CDRequest(
                        method="centered",
                        use_mask=True,
                        object_names=["person"],
                        yolo_model="yolo11n-seg.pt",
                        yolo_model_confidence=0.8,
                        yolo_nms_threshold=0.4,
                    )
                ]
            )
        )

        try:
            names = []
            xywhs = []
            for cropped_img in cropped_response.responses[0].cropped_imgs:
                response = detect_face_service(
                    ClipRecogniseFaceRequest(image_raw=cropped_img)
                )
                names.append(response.name)
                xywhs.append(response.xywh)
                rospy.loginfo(f"Recognised face: {response.name}")

            # Add names to image
            cv2_img = msg_to_cv2_img(cropped_response.responses[0].masked_img)
            for name, xywh in zip(names, xywhs):
                x, y, w, h = xywh[0], xywh[1], xywh[2], xywh[3]
                cv2.rectangle(cv2_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(
                    cv2_img,
                    name,
                    (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
            debug_pub.publish(cv2_img_to_msg(cv2_img))
        except Exception as e:
            rospy.loginfo(e)
            continue

    rospy.spin()
