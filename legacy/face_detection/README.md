# face_detection

The face_detection package

This package is maintained by:
- [nicole](mailto:nicole@todo.todo)

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

#### `FaceDetection`

Request

| Field | Type | Description |
|:-:|:-:|---|
| image_raw | sensor_msgs/Image |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| detected_objects | lasr_vision_msgs/Detection[] |  |

#### `FaceDetectionPCL`

Request

| Field | Type | Description |
|:-:|:-:|---|
| cloud | sensor_msgs/PointCloud2 |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| detections | face_detection/DetectionPCL[] |  |


### Actions

This package has no actions.
