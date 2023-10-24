# lasr_object_detection_yolo

The lasr_object_detection_yolo package

This package is maintained by:
- [jared](mailto:jared@todo.todo)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- rospy (build)
- std_msgs (build)
- rospy (exec)
- std_msgs (exec)
- lasr_vision_msgs

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

This package has no messages.

### Services

#### `YoloDetection`

Request

| Field | Type | Description |
|:-:|:-:|---|
| image_raw | sensor_msgs/Image |  |
| dataset | string |  |
| confidence | float32 |  |
| nms | float32 |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| detected_objects | lasr_vision_msgs/Detection[] |  |


### Actions

This package has no actions.
