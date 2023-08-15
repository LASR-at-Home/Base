#!/usr/bin/env python3
import cv2, matplotlib.pyplot as plt

import torch
from PIL import Image

from torchvision.transforms import functional as F

import glob
from common_tools import get_model
import os, sys

FWD = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.abspath(os.path.join(FWD, "../src/choosing_wait_position")))
os.chdir(os.path.abspath(os.path.join(FWD, 'models')))


def make_prediction(image_path):

    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    import os
    print(os.getcwd())
    model = get_model(1, weights_path='keypointsrcnn_weights.pth')
    # common / navigation / choosing_wait_position / src / choosing_wait_position / final_lift_key_point / models / keypointsrcnn_weights.pth

    origin_list = glob.glob(image_path)
    print(origin_list)

    for i, img_name in enumerate(origin_list):
        original_img = Image.open(img_name)

        img = F.to_tensor(original_img)
        img = torch.unsqueeze(img,dim=0)

        model.eval()
        with torch.no_grad():
            predictions = model(img.to(device))[0]
            predict_boxes = predictions["boxes"].to("cpu").numpy()
            predict_classes = predictions["labels"].to("cpu").numpy()
            predict_keypoints = predictions["keypoints"].to("cpu").numpy()
            bbox = predict_boxes[0]
            keypoint = predict_keypoints[0]

        return bbox,keypoint


def visualise_predictions(image, bbox, keypoint): 
    print("bbox:", bbox)
    print("keypoint: ", keypoint)


    start_point = (int(bbox[0]),int(bbox[1]))
    end_point = (int(bbox[2]),int(bbox[3]))
    target_pos = (int(keypoint[0][0]), int(keypoint[0][1]))
    window_name = "prediction"
    color = (255,0,0)
    thickness = 2

    image = cv2.rectangle(image, start_point,end_point,color,thickness)
    image = cv2.circle(image, target_pos, 5, (255,0,0), 10)


    plt.figure(figsize=(40,40))
    plt.imshow(image)
    plt.show()


#EXAMPLE: 

print(os.getcwd())
#We create an image object using the image path: 
image_path = '../834.jpg'
# image_sample = Image.open(image_path)
# base/src/Base/common/navigation/choosing_wait_position/src/choosing_wait_position/final_lift_key_point/data/834.jpg

#Then we predict the bounding box and the keypoint using the make predictions class
bbox, keypoint = make_prediction(image_path)


#We create an image object using the image path:
image = cv2.imread(image_path)
#We pass the bounding boxes and keypoints to this function to visualise: 
visualise_predictions(image, bbox,keypoint)