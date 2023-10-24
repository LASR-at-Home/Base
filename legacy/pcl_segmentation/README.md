# pcl_segmentation

The pcl_segmentation package

This package is maintained by:
- [jared](mailto:jared@todo.todo)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- roscpp (build)
- roscpp (exec)

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

#### `Centroid`

Request

| Field | Type | Description |
|:-:|:-:|---|
| points | sensor_msgs/PointCloud2 |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| centroid | geometry_msgs/PointStamped |  |

#### `SegmentCuboid`

Request

| Field | Type | Description |
|:-:|:-:|---|
| points | sensor_msgs/PointCloud2 |  |
| min | geometry_msgs/Point |  |
| max | geometry_msgs/Point |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| points | sensor_msgs/PointCloud2 |  |

#### `MaskFromCuboid`

Request

| Field | Type | Description |
|:-:|:-:|---|
| points | sensor_msgs/PointCloud2 |  |
| min | geometry_msgs/Point |  |
| max | geometry_msgs/Point |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| mask | sensor_msgs/Image |  |

#### `SegmentBB`

Request

| Field | Type | Description |
|:-:|:-:|---|
| points | sensor_msgs/PointCloud2 |  |
| x1 | int32 |  |
| y1 | int32 |  |
| x2 | int32 |  |
| y2 | int32 |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| points | sensor_msgs/PointCloud2 |  |


### Actions

This package has no actions.
