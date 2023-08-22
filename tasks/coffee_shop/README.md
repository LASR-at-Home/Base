# coffee_shop

The coffee_shop package

This package is maintained by:
- [Jared Swift](mailto:jared@todo.todo)
- [Peter Tisnikar](mailto:peter@todo.todo)
- [Paul Makles](mailto:me@insrt.uk)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- catkin_virtualenv (build)
- cv_bridge3

This packages requires Python 3.10 to be present.

This package has 61 Python dependencies:
- [rosnumpy](https://pypi.org/project/rosnumpy)==0.0.5.2
- [numpy](https://pypi.org/project/numpy)==1.25.2
- [scipy](https://pypi.org/project/scipy)==1.11.1
- [shapely](https://pypi.org/project/shapely)==2.0.1
- .. and 57 sub dependencies

Ask the package maintainer to write or create a blank `doc/PREREQUISITES.md` for their package!

## Usage

Ask the package maintainer to write a `doc/USAGE.md` for their package!

## Example

Ask the package maintainer to write a `doc/EXAMPLE.md` for their package!

## Technical Overview

Ask the package maintainer to write a `doc/TECHNICAL.md` for their package!

## ROS Definitions

### Launch Files

#### `sim`

No description provided.

#### `phase3`

No description provided.

#### `make_order`

No description provided.

#### `demo`

No description provided.

#### `check_table`

No description provided.

#### `wait_for_person`

No description provided.


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
