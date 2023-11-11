import cv2
import numpy as np
from PIL import Image
import tensorflow as tf
from tf_bodypix.api import download_model, load_model, BodyPixModelPaths

# capture from webcam
cap = cv2.VideoCapture(0)

# load model
bodypix_model = load_model(
    download_model(BodyPixModelPaths.MOBILENET_FLOAT_50_STRIDE_16)
)

while cap.isOpened():
    success, frame = cap.read()

    if success:
        # run prediction on camera frame
        result = bodypix_model.predict_single(frame)

        # extract mask with minimum confidence
        mask = result.get_mask(threshold=0.75)

        # generate mask of only face
        mask2 = result.get_part_mask(mask=mask, part_names=['left_face', 'right_face']).squeeze()

        # mask out non-face areas
        frame[mask2 == 0] = 0

        # preview
        cv2.imshow("Preview", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        print("fail!")
        break

cap.release()
cv2.destroyAllWindows()
