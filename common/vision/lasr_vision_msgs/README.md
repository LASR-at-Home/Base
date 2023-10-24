# lasr_vision_msgs

Messages required for vision processing

This package is maintained by:
- [Paul Makles](mailto:me@insrt.uk)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- message_generation (build)
- message_runtime (exec)
- sensor_msgs (build)
- sensor_msgs (exec)

Ask the package maintainer to write or create a blank `doc/PREREQUISITES.md` for their package!

## Usage

Ask the package maintainer to write a `doc/USAGE.md` for their package!

## Example

Ask the package maintainer to write a `doc/EXAMPLE.md` for their package!

## Technical Overview

Ask the package maintainer to write a `doc/TECHNICAL.md` for their package!

## ROS Definitions

### Launch Files

This package has no launch files.

### Messages

#### `Detection`

| Field | Type | Description |
|:-:|:-:|---|
| name | string | Detected Object Class |
| confidence | float32 | How certain we are this is what we think it is |
| xywh | int32[] | Bounding box mask defined in pixel-space |
| xyseg | int32[] | Segmentation mask defined in pixel-space<br/><br/>This will be empty if a segmentation model was not used |


### Services

#### `YoloDetection`

Request

| Field | Type | Description |
|:-:|:-:|---|
| image_raw | sensor_msgs/Image | Image to run inference on |
| dataset | string | YOLO model to use |
| confidence | float32 | How certain the detection should be to include |
| nms | float32 | Non-maximum Supression<br/><br/>Guiding value to remove overlapping bounding boxes |

Response

| Field | Type | Description |
|:-:|:-:|---|
| detected_objects | lasr_vision_msgs/Detection[] | Detection result |


### Actions

This package has no actions.
