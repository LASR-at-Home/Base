# tf_module

The tf_module package

This package is maintained by:
- [nicole](mailto:nicole@todo.todo)

## Prerequisites

This package depends on the following ROS packages:
- geometry_msgs (build)
- message_generation (build)
- roscpp (build)
- rospy (build)
- sensor_msgs (build)
- std_msgs (build)
- geometry_msgs (exec)
- roscpp (exec)
- rospy (exec)
- sensor_msgs (exec)
- std_msgs (exec)
- catkin (buildtool)

Ask the package maintainer to write or create a blank `doc/PREREQUISITES.md` for their package!

## Usage

Ask the package maintainer to write a `doc/USAGE.md` for their package!

## Example

Ask the package maintainer to write a `doc/EXAMPLE.md` for their package!

## Technical Overview

Ask the package maintainer to write a `doc/TECHNICAL.md` for their package!

## ROS Definitions

### Launch Files

#### `tf_transforms`

No description provided.


### Messages

This package has no messages.

### Services

#### `ApplyTransform`

Request

| Field | Type | Description |
|:-:|:-:|---|
| points | geometry_msgs/Point[] | point we want to transform |
| transform | geometry_msgs/TransformStamped | the transform we want to use |

Response

| Field | Type | Description |
|:-:|:-:|---|
| new_points | geometry_msgs/Point[] | the transformed point |

#### `TfTransform`

Request

| Field | Type | Description |
|:-:|:-:|---|
| pose_array | geometry_msgs/PoseArray |  |
| pointcloud | sensor_msgs/PointCloud2 |  |
| point | geometry_msgs/PointStamped |  |
| target_frame | std_msgs/String |  |
| source_frame | std_msgs/String |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| target_pose_array | geometry_msgs/PoseArray |  |
| target_pointcloud | sensor_msgs/PointCloud2 |  |
| target_point | geometry_msgs/PointStamped |  |
| translation_and_rotation | geometry_msgs/Pose |  |

#### `LatestTransform`

Request

| Field | Type | Description |
|:-:|:-:|---|
| from_frame | string | source frame |
| target_frame | string | target frame |

Response

| Field | Type | Description |
|:-:|:-:|---|
| transform | geometry_msgs/TransformStamped | transform |

#### `BaseTransform`

Request

| Field | Type | Description |
|:-:|:-:|---|
| points | geometry_msgs/Point[] | point we want to transform |
| frame | std_msgs/String | the frame of the point |
| target_frame | std_msgs/String | the frame we want to transform the point to |

Response

| Field | Type | Description |
|:-:|:-:|---|
| new_points | geometry_msgs/PointStamped[] | the transformed point |


### Actions

This package has no actions.
