#!/usr/bin/env python3

import cv2
import numpy as np
from PIL import Image
import tensorflow as tf
import matplotlib.pyplot as plt
from tf_bodypix.api import download_model, load_model, BodyPixModelPaths

# capture from webcam
cap = cv2.VideoCapture(0)

# load model
bodypix_model = load_model(
    # uncomment to use MobileNet instead:
    # download_model(BodyPixModelPaths.MOBILENET_FLOAT_50_STRIDE_16)
    download_model(BodyPixModelPaths.RESNET50_FLOAT_STRIDE_16)
)

while cap.isOpened():
    success, frame = cap.read()

    if success:
        # run prediction on camera frame
        result = bodypix_model.predict_single(frame)

        # extract mask with minimum confidence
        mask = result.get_mask(threshold=0.75)
        
        # generate coloured mask
        coloured_mask = result.get_colored_part_mask(mask).astype(np.uint8)
        
        # draw poses on top
        from tf_bodypix.draw import draw_poses
        poses = result.get_poses()
        coloured_mask = draw_poses(
            coloured_mask.copy(),
            poses,
            keypoints_color=(255, 100, 100),
            skeleton_color=(100, 100, 255),
        )

        # preview
        cv2.imshow("Preview", coloured_mask)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        print("fail!")
        break

cap.release()
cv2.destroyAllWindows()
