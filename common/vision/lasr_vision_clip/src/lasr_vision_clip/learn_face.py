#!/usr/bin/env python3
import os
import cv2
import rospy
from typing import Dict
import numpy as np
import rospkg
from lasr_vision_msgs.srv import (
    ClipRecogniseFaceRequest,
    ClipRecogniseFaceResponse,
    ClipLearnFace,
    ClipRecogniseFace,
    ClipLearnFaceRequest,
    ClipLearnFaceResponse,
)
from sensor_msgs.msg import Image
from cv2_img import msg_to_cv2_img, cv2_img_to_msg
from lasr_vision_clip import load_face_model, encode_img, infer


class FaceService:
    def __init__(self, similarity_threshold: float = 6.0) -> None:
        self._face_classifier = cv2.CascadeClassifier(
            os.path.join(
                rospkg.RosPack().get_path("lasr_vision_clip"),
                "data",
                "haarcascade_frontalface_default.xml",
            )
        )
        self.learned_faces: Dict[str, np.ndarray] = {}
        self._similarity_threshold = similarity_threshold
        self.processor, self.model = load_face_model()
        self._face_pub = rospy.Publisher("/clip/face_detection", Image, queue_size=1)

        rospy.Service("/vision/face_detection", ClipRecogniseFace, self.face_detection)
        rospy.Service("/vision/learn_face", ClipLearnFace, self.learn_face)

        rospy.loginfo("Face detector service started")

    def _detect_faces(self, img: np.ndarray):
        faces = self._face_classifier.detectMultiScale(
            img, 1.1, minNeighbors=5, minSize=(10, 10)
        )
        return faces

    def face_detection(
        self, req: ClipRecogniseFaceRequest
    ) -> ClipRecogniseFaceResponse:
        img = req.image_raw
        cv2_img = msg_to_cv2_img(img)
        # cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
        try:
            faces = self._detect_faces(cv2_img)

            # Assume only one face in image
            encoded_face = None
            closest_name = "Unknown"
            min_dist = float("inf")
            min_xywh = None
            for x, y, w, h in faces:
                cv2_face = cv2_img[y : y + h, x : x + w]
                # cv2_face = cv2.cvtColor(cv2_face, cv2.COLOR_GRAY2BGR)
                face_msg = cv2_img_to_msg(cv2_face)
                self._face_pub.publish(face_msg)
                encoded_face = infer(
                    cv2_img_to_msg(cv2_img), self.processor, self.model
                )
                encoded_face = encoded_face.flatten()
                for name, face in self.learned_faces.items():
                    distance = np.linalg.norm(encoded_face - face)
                    rospy.loginfo(f"Distance to {name} : {distance}")
                    if distance < min_dist:
                        min_dist = distance
                        min_xywh = [x, y, w, h]
                        closest_name = name
            return ClipRecogniseFaceResponse(
                name=closest_name, distance=min_dist, xywh=min_xywh
            )
        except Exception as e:
            rospy.loginfo(e)
            return ClipRecogniseFaceResponse(name="Unknown", distance=None, xywh=None)

    def learn_face(self, request: ClipLearnFaceRequest) -> ClipLearnFaceResponse:
        imgs = request.raw_imgs

        embedding_vectors = []
        for img in imgs:
            cv2_img = msg_to_cv2_img(img)
            # cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
            rospy.loginfo(f"Image shape: {cv2_img.shape}")
            try:
                faces = self._detect_faces(cv2_img)
            except Exception as e:  # No face detected
                rospy.loginfo(e)
                continue
            for x, y, w, h in faces:
                cv2_face = cv2_img[y : y + h, x : x + w]
                # cv2_face = cv2.cvtColor(cv2_face, cv2.COLOR_GRAY2BGR)
                face_msg = cv2_img_to_msg(cv2_face)
                self._face_pub.publish(face_msg)
                encoded_face = infer(
                    cv2_img_to_msg(cv2_img), self.processor, self.model
                )
                encoded_face = encoded_face.flatten()
                embedding_vectors.append(encoded_face)

        embedding_vectors = np.array(embedding_vectors)
        embedding_vector = np.mean(embedding_vectors, axis=0)
        self.learned_faces[request.name] = embedding_vector
        rospy.loginfo(f"Learned {request.name}")

        return ClipLearnFaceResponse()
