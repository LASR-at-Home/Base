# open_vocabulary_models

ROS2 service wrapper for open-vocabulary models (OWLv2, DINO, VitSam, YOLOWorld).

This package exposes services under `open_vocab/` that other nodes can call to request detections and masks.

Currently the node scaffolding includes a YOLOWorld-based detector as an example. After building the workspace the srv/msg will be generated and the node can be used via ROS2 service calls.
