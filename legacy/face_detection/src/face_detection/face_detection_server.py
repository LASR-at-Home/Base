#!/usr/bin/env python3
from genericpath import exists
import rospy
import rospkg

from face_detection.srv import FaceDetection, FaceDetectionResponse, \
    FaceDetectionRequest
from lasr_vision_msgs.msg import Detection
import os
import numpy as np
import cv2
import pickle
import imutils
from cv_bridge3 import CvBridge

NET = os.path.join(rospkg.RosPack().get_path('face_detection'),'nn4', "nn4.small2.v1.t7")
PROTO_PATH = os.path.join(rospkg.RosPack().get_path('face_detection'), 'caffe_model', "deploy.prototxt")
MODEL_PATH = os.path.join(rospkg.RosPack().get_path('face_detection'), 'caffe_model',
                          "res10_300x300_ssd_iter_140000.caffemodel")
OUTPUT_PATH = os.path.join(rospkg.RosPack().get_path('face_detection'), "output")

CONFIDENCE_THRESHOLD = 0.5

#
# def delete_model():
#     if os.path.exists(os.path.join(OUTPUT_PATH, "recognizer.pickle")):
#         os.path.remove(os.path.join(OUTPUT_PATH, "recognizer.pickle"))
#     if os.path.exists(os.path.join(OUTPUT_PATH, "le.pickle")):
#         os.path.remove(os.path.join(OUTPUT_PATH, "le.pickle"))


class FaceDetectionServer:
    """
    A Server for performing face detection and classification with OpenCV.
    """

    def __init__(self):

        # Load network and models.
        self.detector = cv2.dnn.readNetFromCaffe(PROTO_PATH, MODEL_PATH)
        self.embedder = cv2.dnn.readNetFromTorch(NET)

        # Bridge for conversion between cv2 and sensor_msgs/Image, and vice versa.
        self.bridge = CvBridge()
        # delete_model()
        self.recognizer = None
        self.le = None

    def load_model(self):
        if not os.path.exists(os.path.join(OUTPUT_PATH, "recognizer.pickle")):
            self.recognizer = None
        else:
            with open(os.path.join(OUTPUT_PATH, "recognizer.pickle"), "rb") as fp:
                self.recognizer = pickle.loads(fp.read())
        if not os.path.exists(os.path.join(OUTPUT_PATH, "le.pickle")):
            self.le = None
        else:
            with open(os.path.join(OUTPUT_PATH, "le.pickle"), "rb") as fp:
                self.le = pickle.loads(fp.read())
                print(self.le.classes_)

        if self.recognizer and self.le:
            return True

    def __call__(self, req):
        """
        Core method of server.
        """

        if not self.recognizer or not self.le:
            if not self.load_model():
                raise rospy.ServiceException("No model to load from.")

        # Construct empty response.
        if isinstance(req, FaceDetectionRequest):
            response = FaceDetectionResponse()

            # Convert sensor_msgs/Image to cv2 image.
            print('i am here')
            cv_image = self.bridge.imgmsg_to_cv2_np(req.image_raw)

        # else:
        #     response = FaceDetectionPCLResponse()
        #
        #     # Extract rgb image from pointcloud
        #     cv_image = np.fromstring(req.cloud.data, dtype=np.uint8)
        #     cv_image = cv_image.reshape(req.cloud.height, req.cloud.width, 32)
        #     cv_image = cv_image[:, :, 16:19]
        #
        #     # Ensure array is contiguous
        #     cv_image = np.ascontiguousarray(cv_image, dtype=np.uint8)

        # Resize for input to the network.
        cv_image = imutils.resize(cv_image, width=600)
        h, w = cv_image.shape[:2]

        # Construct a blob from the image.
        blob = cv2.dnn.blobFromImage(
            cv2.resize(cv_image, (300, 300)),
            1.0, (300, 300),
            (104.0, 177.0, 123.0),
            swapRB=False, crop=False
        )

        # Apply OpenCV's deep learning-based face detector to localize
        # faces in the input image
        self.detector.setInput(blob)
        detections = self.detector.forward()

        print(detections, 'the face detections')

        # Iterate detections
        for detection in detections[0][0]:

            # Extract confidence.
            confidence = detection[2]

            # Ensure confidence of detection is above specified threshold.
            if confidence > CONFIDENCE_THRESHOLD:

                # Compute bounding box coordinates.
                face_bb = detection[3:7] * np.array([w, h, w, h])
                x1, y1, x2, y2 = face_bb.astype("int")

                # Extract face.
                face = cv_image[y1:y2, x1:x2]

                face_width, face_height = face.shape[:2]

                # Ensure face width and height are sufficiently large.
                if face_width < 20 or face_height < 20: continue

                # Construct a blob from the face.
                faceBlob = cv2.dnn.blobFromImage(
                    face, 1.0 / 255, (96, 96),
                    (0, 0, 0),
                    swapRB=True, crop=False
                )

                # Pass blob through face embedding model
                # to obtain the 128-d quantification of the face.
                self.embedder.setInput(faceBlob)
                vec = self.embedder.forward()

                # Perform classification to recognise the face.
                predictions = self.recognizer.predict_proba(vec)[0]
                j = np.argmax(predictions)
                prob = predictions[j]
                name = self.le.classes_[j]
                print(name, prob, 'hello there')

                # Append the detection to the response.
                if isinstance(req, FaceDetectionRequest):
                    response.detected_objects.append(Detection(name, prob, [x1, y1, x2, y2]))
                # else:
                #     centroid = bb_to_centroid(req.cloud, x1, y1, x2 - x1, y2 - y1)
                #     response.detections.append(DetectionPCL(name, prob, [x1, y1, x2, y2], centroid))
        # print('the resp is ', response)
        return response


if __name__ == "__main__":
    rospy.init_node("face_detection_server")
    server = FaceDetectionServer()
    service = rospy.Service('face_detection_server', FaceDetection, server)
    # service_pcl = rospy.Service('face_detection_pcl', FaceDetectionPCL, server)
    rospy.loginfo("Face Detection Service initialised")
    rospy.spin()