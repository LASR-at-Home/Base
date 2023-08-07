# aruco_msgs

The aruco_msgs package

This package is maintained by:
- [Sai Kishor Kothakota](mailto:sai.kishor@pal-robotics.com)

The following people have contributed to this package:
- [Bence Magyar](mailto:bence.magyar@pal-robotics.com)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- geometry_msgs
- std_msgs
- message_generation (build)
- message_runtime (exec)

Ask the package maintainer to write or create a blank `doc/PREREQUISITES.md` for their package!

## Usage

Ask the package maintainer to write a `doc/USAGE.md` for their package!

## Example

Ask the package maintainer to write a `doc/EXAMPLE.md` for their package!

## Technical Overview

Ask the package maintainer to write a `doc/TECHNICAL.md` for their package!

## ROS Definitions

### Messages

#### `Marker`

| Field | Type | Description |
|:-:|:-:|---|
| header | Header |  |
| id | uint32 |  |
| pose | geometry_msgs/PoseWithCovariance |  |
| confidence | float64 |  |

#### `MarkerArray`

| Field | Type | Description |
|:-:|:-:|---|
| header | Header |  |
| markers | aruco_msgs/Marker[] |  |


### Services

This package has no services.

### Actions

This package has no actions.
