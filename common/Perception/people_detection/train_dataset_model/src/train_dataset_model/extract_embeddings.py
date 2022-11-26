#!/usr/bin/env python3
# import the necessary packages
import numpy as np
import argparse
import imutils
from imutils import paths
import pickle
import cv2
import os
import rospkg


# Responsible for using a deep learning feature extractor to generate a 128-D vector describing a face. All faces in
# our dataset will be passed through the neural network to generate embeddings.

def extract():
    # load our serialized face detector from disk
    path_face_detection_model = os.path.join(rospkg.RosPack().get_path('face_detection'), 'caffe_model')
    path_nn = os.path.join(rospkg.RosPack().get_path('face_detection'), 'nn4', "nn4.small2.v1.t7")
    path_dataset = os.path.join(rospkg.RosPack().get_path('create_dataset'), "dataset")
    path_output = os.path.join(rospkg.RosPack().get_path('face_detection'), "output")

    print("[INFO] loading face detector...")
    proto_path = os.path.join(path_face_detection_model, "deploy.prototxt")
    model_path = os.path.join(path_face_detection_model,
                              "res10_300x300_ssd_iter_140000.caffemodel")
    detector = cv2.dnn.readNetFromCaffe(proto_path, model_path)
    # load our serialized face embedding model from disk
    print("[INFO] loading face recognizer...")
    embedder = cv2.dnn.readNetFromTorch(path_nn)

    # grab the paths to the input images in our dataset
    print("[INFO] quantifying faces...")
    image_paths = list(paths.list_images(path_dataset))
    print(image_paths)
    # initialize our lists of extracted facial embeddings and
    # corresponding people names
    known_embeddings = []
    known_names = []
    # initialize the total number of faces processed
    total = 0

    # loop over the image paths
    for (i, imagePath) in enumerate(image_paths):
        # extract the person name from the image path
        print("[INFO] processing image {}/{}".format(i + 1,
                                                     len(image_paths)))
        name = imagePath.split(os.path.sep)[-2]
        # load the image, resize it to have a width of 600 pixels (while
        # maintaining the aspect ratio), and then grab the image
        # dimensions
        image = cv2.imread(imagePath)
        print(imagePath)
        image = imutils.resize(image, width=600)
        (h, w) = image.shape[:2]

        # construct a blob from the image
        image_blob = cv2.dnn.blobFromImage(
            cv2.resize(image, (300, 300)), 1.0, (300, 300),
            (104.0, 177.0, 123.0), swapRB=False, crop=False)
        # apply OpenCV's deep learning-based face detector to localize
        # faces in the input image
        detector.setInput(image_blob)
        detections = detector.forward()
        # ensure at least one face was found
        if len(detections) > 0:
            # we're making the assumption that each image has only ONE
            # face, so find the bounding box with the largest probability
            i = np.argmax(detections[0, 0, :, 2])
            confidence = detections[0, 0, i, 2]
            # ensure that the detection with the largest probability also
            # means our minimum probability test (thus helping filter out
            # weak detections)
            if confidence > 0.5:
                # compute the (x, y)-coordinates of the bounding box for
                # the face
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")

                # extract the face ROI and grab the ROI dimensions
                face = image[startY:endY, startX:endX]
                (fH, fW) = face.shape[:2]
                # ensure the face width and height are sufficiently large
                if fW < 20 or fH < 20:
                    continue
                # construct a blob for the face ROI, then pass the blob
                # through our face embedding model to obtain the 128-d
                # quantification of the face
                faceBlob = cv2.dnn.blobFromImage(face, 1.0 / 255,
                                                 (96, 96), (0, 0, 0), swapRB=True, crop=False)
                embedder.setInput(faceBlob)
                vec = embedder.forward()
                # add the name of the person + corresponding face
                # embedding to their respective lists
                known_names.append(name)

                known_embeddings.append(vec.flatten())
                total += 1
                # dump the facial embeddings + names to disk
    print("[INFO] serializing {} encodings...".format(total))
    print('YO I KNOW THESE PEEPS: ', known_names)
    data = {"embeddings": known_embeddings, "names": known_names}
    f = open(path_output + "/embeddings.pickle", "wb")
    f.write(pickle.dumps(data))
    f.close()