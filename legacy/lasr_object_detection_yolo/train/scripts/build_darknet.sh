#!/bin/bash
git clone https://github.com/AlexeyAB/darknet
make -C darknet GPU=$USE_GPU OPENCV=$USE_OPENCV CUDNN=$USE_CUDNN OPENMP=$USE_OPENMP
wget -P pretrained_weights https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.conv.137