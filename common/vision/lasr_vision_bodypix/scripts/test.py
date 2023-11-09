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
    # download_model(BodyPixModelPaths.MOBILENET_FLOAT_50_STRIDE_16)
    download_model(BodyPixModelPaths.RESNET50_FLOAT_STRIDE_16)
)

while cap.isOpened():
    success, frame = cap.read()

    if success:
        # im = Image.fromarray(frame, 'RGB')
        # img_array = np.array(im)

        # expand dimensions to match the 4D Tensor shape
        # img_array = np.expand_dims(img_array, axis=0)
        # img_array = tf.keras.preprocessing.image.img_to_array(img_array)

        result = bodypix_model.predict_single(frame)
        mask = result.get_mask(threshold=0.75)
        mask2 = result.get_part_mask(mask=mask, part_names=['torso_front', 'torso_back']).squeeze()
        
        aaa = result.get_colored_part_mask(mask).astype(np.uint8)
        from tf_bodypix.draw import draw_poses  # utility function using OpenCV

        poses = result.get_poses()
        aaa = draw_poses(
            aaa.copy(),  # create a copy to ensure we are not modifing the source image
            poses,
            keypoints_color=(255, 100, 100),
            skeleton_color=(100, 100, 255),
        )

        # cv2_img = cv2.cvtColor(aaa, cv2.COLOR_RGB2BGR)
        # aaa_im = np.array(Image.fromarray(aaa, 'RGB'))

        # np.save("yay", mask2)
        # mask2 = (1 - mask2.squeeze())
        # mask2 = mask2.squeeze()

        # mask_image = Image.fromarray(mask2.astype(    .uint8))
        # mask2 = mask2.numpy()
        # print(mask2.shape)
        #cv2.imshow("gghd", mask_image)

        # height, width, _ = frame.shape
        # object_image = np.zeros((height, width, 4), dtype=np.uint8)
        # object_image[:,:,0:3] = frame
        # object_image[:, :, 3] = mask2
        
        # mask2_resized = cv2.resize(mask2.squeeze(), (frame.shape[1], frame.shape[0]))
        # cv2.bitwise_and(frame, mask2)

        # print(frame.shape, mask2.shape)
        frame[mask2 == 0] = 0

        # HISTORGRAM!!
        # red, green, blue = frame[:,:,0].flatten(), frame[:,:,1].flatten(), frame[:,:,2].flatten()
        # plt.figure(figsize=(12, 12))
        # plt.hist(red,   bins=256, density=False, color='red', alpha=0.5)
        # plt.hist(green, bins=256, density=False, color='green', alpha=0.4)
        # plt.hist(blue,  bins=256, density=False, color='blue', alpha=0.3)

        # plt.xticks(ticks=np.linspace(0, 1, 17), labels=range(0, 257, 16))

        # plt.title("hgfhgdf")
        # plt.ylabel("count")
        # plt.xlabel("intensity")

        # plt.show()

        cv2.imshow("ogjgfijhogf", aaa)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        print("fail!")
        break

cap.release()
cv2.destroyAllWindows()
