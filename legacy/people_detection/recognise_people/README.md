# recognise_people

The recognise_people package

This package is maintained by:
- [nicole](mailto:nicole@todo.todo)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- rospy (build)
- std_msgs (build)
- rospy (exec)
- std_msgs (exec)
- lasr_perception_server

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

#### `RecognisePeople`

Request

| Field | Type | Description |
|:-:|:-:|---|
| detected_objects_yolo | lasr_perception_server/Detection[] |  |
| detected_objects_opencv | lasr_perception_server/Detection[] |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| detected_objects | lasr_perception_server/Detection[] |  |


### Actions

This package has no actions.
