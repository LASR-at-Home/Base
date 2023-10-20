# common_math

The common_math package

This package is maintained by:
- [jared](mailto:jared@todo.todo)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- actionlib (build)
- actionlib_msgs (build)
- geometry_msgs (build)
- message_generation (build)
- rospy (build)
- std_msgs (build)
- std_srvs (build)
- actionlib (exec)
- actionlib_msgs (exec)
- geometry_msgs (exec)
- rospy (exec)
- std_msgs (exec)
- std_srvs (exec)
- face_detection (exec)

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

#### `TfTransform`

Request

| Field | Type | Description |
|:-:|:-:|---|
| pose_array | geometry_msgs/PoseArray |  |
| pointcloud | sensor_msgs/PointCloud2 |  |
| point | geometry_msgs/PointStamped |  |
| target_frame | std_msgs/String |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| target_pose_array | geometry_msgs/PoseArray |  |
| target_pointcloud | sensor_msgs/PointCloud2 |  |
| target_point | geometry_msgs/PointStamped |  |


### Actions

This package has no actions.
