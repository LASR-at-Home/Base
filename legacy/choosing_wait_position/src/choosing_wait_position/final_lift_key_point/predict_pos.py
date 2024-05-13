#!/usr/bin/env python3
import random

import cv2, matplotlib.pyplot as plt

import torch
from PIL import Image
import numpy as np
from torchvision.transforms import functional as F
from narrow_space_navigation.waypoints import *

import glob
from choosing_wait_position.final_lift_key_point.common_tools import get_model
import os, sys

from lift.defaults import PLOT_SAVE, PLOT_SHOW, TEST, DEBUG_PATH

FWD = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.abspath(os.path.join(FWD, "../src/choosing_wait_position")))
os.chdir(os.path.abspath(os.path.join(FWD, 'models')))


def make_prediction(image_path):
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    model = get_model(1, weights_path='keypointsrcnn_weights.pth')

    origin_list = glob.glob(image_path)
    print(image_path)

    # origin_list = [image_path]
    print(origin_list)

    for i, img_name in enumerate(origin_list):
        original_img = Image.open(img_name)

        plt.imshow(original_img, cmap='gray')

        if PLOT_SHOW:
            plt.show()
        if PLOT_SAVE:
            plt.savefig(DEBUG_PATH + "/predict_pos_zoe" + str(TEST) + ".jpg")

        img = F.to_tensor(original_img)
        img = torch.unsqueeze(img, dim=0)

        model.eval()
        with torch.no_grad():
            predictions = model(img.to(device))[0]
            predict_boxes = predictions["boxes"].to("cpu").numpy()
            print("predict boxes: ", predict_boxes)
            predict_classes = predictions["labels"].to("cpu").numpy()
            predict_keypoints = predictions["keypoints"].to("cpu").numpy()
            try:
                bbox = predict_boxes[0]
                keypoint = predict_keypoints[0]
            except:
                return None, None

        return bbox, keypoint


def visualise_predictions(image, bbox, keypoint):
    print("bbox:", bbox)
    print("keypoint: ", keypoint)

    window_name = "prediction"
    color = (255, 0, 0)
    thickness = 1

    try:
        start_point = (int(bbox[0]), int(bbox[1]))
        end_point = (int(bbox[2]), int(bbox[3]))
        image = cv2.rectangle(image, start_point, end_point, color, thickness)
        target_pos = (int(keypoint[0][0]), int(keypoint[0][1]))
        image = cv2.circle(image, target_pos, 1, (255, 0, 0), 1)
    except Exception as e:
        print(e)

    plt.imshow(image, cmap='gray', aspect='auto')
    plt.title('visualisation of the prediction')
    if PLOT_SHOW:
        plt.show()
    if PLOT_SAVE:
        plt.savefig(DEBUG_PATH + "/predict_pos_viz_test_test" + str(TEST) + ".jpg")

# from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
# from tiago_controllers.controllers.controllers import Controllers
#
# #EXAMPLE:
# import rospy
# rospy.init_node('predict_pos')
# print(os.getcwd())
# #We create an image object using the image path:
# w = Waypoint()
# warped, analysis, M = w.get_lift_information()
# plt.imshow(warped, cmap='gray')
# plt.show()
# image = Image.fromarray(warped)
# image.save("test.jpg")
#
# image_path = "test.jpg"
# # image_path = '../834.jpg'
#
# print(image_path)
#
# # image_sample = Image.open(image_path)
# # base/src/Base/common/navigation/choosing_wait_position/src/choosing_wait_position/final_lift_key_point/data/834.jpg
#
# #Then we predict the bounding box and the keypoint using the make predictions class
# bbox, keypoint = make_prediction(image_path)
# print(f"keypoint: {keypoint}")
# print(type(keypoint))
# global_points = w.local_to_global_points(M=M, points=keypoint)
# p = Pose()
# p.position.x =  global_points[0][0]
# p.position.y =  global_points[0][1]
# p.orientation.w = 1
# c = Controllers()
# c.base_controller.sync_to_pose(p)
#
#
#
#
# #We create an image object using the image path:
# image = cv2.imread(image_path)
# #We pass the bounding boxes and keypoints to this function to visualise:
# visualise_predictions(image, bbox,keypoint)
