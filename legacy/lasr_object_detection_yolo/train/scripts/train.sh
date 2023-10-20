#!/bin/bash
DATASET=$1
python scripts/generate_config_files.py $DATASET
./darknet/darknet detector train config/$DATASET/$DATASET.data config/$DATASET/yolov4.cfg pretrained_weights/yolov4.conv.137