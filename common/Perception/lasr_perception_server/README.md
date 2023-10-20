# lasr_perception_server

The lasr_perception_server package

This package is maintained by:
- [jared](mailto:jared@todo.todo)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- rospy (build)
- std_msgs (build)
- rospy (exec)
- std_msgs (exec)

Ask the package maintainer to write or create a blank `doc/PREREQUISITES.md` for their package!

## Usage

Ask the package maintainer to write a `doc/USAGE.md` for their package!

## Example

Ask the package maintainer to write a `doc/EXAMPLE.md` for their package!

## Technical Overview

Ask the package maintainer to write a `doc/TECHNICAL.md` for their package!

## ROS Definitions

### Launch Files

#### `perception_server`

No description provided.


### Messages

#### `Detection`

| Field | Type | Description |
|:-:|:-:|---|
| name | string | Detected Object Class |
| confidence | float32 | How certain we are this is what we think it is |
| xywh | int32[] | Bounding box mask defined in pixel-space |
| xyseg | int32[] | Segmentation mask defined in pixel-space<br/><br/>This will be empty if a segmentation model was not used |

#### `Detections`

| Field | Type | Description |
|:-:|:-:|---|
| name | string |  |
| confidence | float32 |  |
| xywh | int32[] |  |


### Services

#### `DetectImage`

Request

| Field | Type | Description |
|:-:|:-:|---|
| image | sensor_msgs/Image[] |  |
| dataset | string |  |
| confidence | float32 |  |
| nms | float32 |  |
| filter | string[] |  |
| task | string |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| detected_objects | lasr_perception_server/Detection[] |  |

#### `DetectImages`

Request

| Field | Type | Description |
|:-:|:-:|---|
| image | sensor_msgs/Image[] |  |
| dataset | string |  |
| confidence | float32 |  |
| nms | float32 |  |
| filter | string[] |  |
| task | string |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| detected_objects | lasr_perception_server/Detection[] |  |


### Actions

This package has no actions.
