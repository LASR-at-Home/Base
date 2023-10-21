#!/usr/bin/env python3
import rospy
import rospkg

import torch
import torchvision.transforms as transforms

import cv2
from PIL import Image as PIL_Image
import numpy as np

import os
import time

from darknet_pytorch.darknet import Darknet
from darknet_pytorch.utils import post_processing

from lasr_vision_msgs.srv import YoloDetection, YoloDetectionResponse
from lasr_vision_msgs.msg import Detection

import nvidia_smi


MODEL_ROOT = os.path.join(rospkg.RosPack().get_path('lasr_object_detection_yolo'), 'models')


def transform():
    return transforms.Compose([
        transforms.Resize((416, 416)),
        transforms.ToTensor(),
    ])

class YoloObjectDetectionServer():

    def __init__(self):

        self.model_name = None
        self.yolov4 = None
        self.labels = []
        self.device = 'cpu'

    def load_model(self, model_name):
        model_path = os.path.join(MODEL_ROOT, model_name)

        if os.path.isdir(model_path):
            self.model_name = model_name

            start_time = time.time()

            try:

                with open(os.path.join(model_path, 'classes.txt')) as fp:
                    self.labels = fp.read().strip().splitlines()

            except FileNotFoundError:
                rospy.logerr(f"Couldn't load {self.model_name}, 'classes.txt' does not exist in {model_path}.")
                return False  

            try:

                with open(os.path.join(model_path, 'yolov4.cfg')) as fp:
                    self.yolov4 = Darknet(os.path.join(model_path, 'yolov4.cfg'))

            except FileNotFoundError:
                rospy.logerr(f"Couldn't load {self.model_name}, 'yolov4.cfg' does not exist in {model_path}.")
                return False
            
            try:
                self.yolov4.load_weights(os.path.join(model_path, 'yolov4.weights'))
            
            except FileNotFoundError:
                rospy.logerr(f"Couldn't load {self.model_name}, 'yolov4.weights' does not exist in {model_path}.")
                return False
            
            self.yolov4.eval()
            """
            if torch.cuda.is_available():

                # Initialise nvidia-smi
                nvidia_smi.nvmlInit()

                # Assume a single GPU.
                handle = nvidia_smi.nvmlDeviceGetHandleByIndex(0)

                # Get GPU memory info.
                info = nvidia_smi.nvmlDeviceGetMemoryInfo(handle)

                if info.free / 1024**2 > 1024:
                    self.device = 'cuda'

                # Shutdown nvidia-smi
                nvidia_smi.nvmlShutdown()
            """
            self.device = "cpu"

            print(self.device)
            self.yolov4.to(self.device)

            rospy.loginfo('Time to load {} model: {:.2f} seconds'.format(model_name, time.time() - start_time))

            return True

    def detect(self, req):

        response = YoloDetectionResponse()

        # Only load model if it is not already loaded.
        if not self.model_name == req.dataset:
            if not self.load_model(req.dataset):
                raise rospy.ServiceException(f"Couldn't load model '{req.dataset}'")
        
        # Random colours for bounding boxes.
        np.random.seed(42)
        COLOURS = np.random.randint(0, 255, size=(len(self.labels), 3), dtype="uint8")

        # Perform pre-processing.
        frame = np.frombuffer(req.image_raw.data, dtype=np.uint8).reshape(req.image_raw.height, req.image_raw.width, -1)
        image = PIL_Image.fromarray(frame)
        image = torch.stack([transform()(image)]).to(self.device)

        outputs = self.yolov4(image)

        # net forward and non-mean suppression
        #try:
        #except RuntimeError:
        #    if self.device != 'cpu':
        #        self.device = 'cpu'
                #self.yolov4.to(self.device)
        #        return self.detect(req)
        #    else:
        #        raise rospy.ServiceException("Couldn't use CUDA or CPU....")
        outputs = post_processing(image, req.confidence, req.nms, outputs)

        if not outputs[0] is None:
            for detection in outputs[0]:

                # Class ID of detection.
                idx = np.argmax(detection[5:])

                # Convert bounding box to image co-ordinates.
                bbox = detection[:4]
                bbox[0] *= req.image_raw.width
                bbox[1] *= req.image_raw.height
                bbox[2] *= req.image_raw.width
                bbox[3] *= req.image_raw.height
                x1,y1,x2,y2 = [int(i) for i in bbox]


                obj_conf, class_score = detection[4:6]
                class_pred = int(detection[6])

                # Draw and label bounding box of object in the frame.
                name = self.labels[class_pred]
                confidence = class_score
                x, y, w, h = x1, y1, x2 - x1, y2 - y1
                color = [int(c) for c in COLOURS[idx]]
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                text = "{}: {:.4f}".format(name, confidence)
                cv2.putText(frame, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # Check detection is above the confidence threshold.
                if class_score > req.confidence:
                    xywh = [x1, y1, x2 - x1, y2 - y1]
                    
                    # Append detection.
                    response.detected_objects.append(
                        Detection(
                            name=name,
                            confidence=class_score,
                            xywh=xywh
                        )
                    )
        print(response.detected_objects, 'i am in yolo detect ')
        return response

if __name__ == '__main__':        
    rospy.init_node('yolo_object_detection_server')
    server = YoloObjectDetectionServer()
    serv = rospy.Service('yolo_object_detection_server/detect_objects', YoloDetection, server.detect)
    rospy.loginfo('YOLOv4 object detection service initialised')
    rospy.spin()



