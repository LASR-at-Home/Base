# coffee_shop

The coffee_shop package

This package is maintained by:
- [Jared Swift](mailto:jared@todo.todo)
- [Peter Tisnikar](mailto:peter@todo.todo)
- [Paul Makles](mailto:me@insrt.uk)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- lasr_vision_yolov8
- cv_bridge3

Ask the package maintainer to write or create a blank `doc/PREREQUISITES.md` for their package!

## Usage

Ask the package maintainer to write a `doc/USAGE.md` for their package!

## Example

Ask the package maintainer to write a `doc/EXAMPLE.md` for their package!

## Technical Overview

Ask the package maintainer to write a `doc/TECHNICAL.md` for their package!

## ROS Definitions

### Launch Files

#### `phase3`

No description provided.

| Argument | Default | Description |
|:-:|:-:|---|
| config | test_phase3 |  |
| tablet | true |  |


#### `people_poses`

No description provided.

#### `make_order`

No description provided.

| Argument | Default | Description |
|:-:|:-:|---|
| config | test_make_order |  |
| tablet | true |  |


#### `check_table`

No description provided.

| Argument | Default | Description |
|:-:|:-:|---|
| config | test_check_table |  |


#### `core`

No description provided.

| Argument | Default | Description |
|:-:|:-:|---|
| tablet | true |  |
| whisper_matcher | by-index |  |
| whisper_device_param | 18 |  |
| rasa_model | $(find lasr_rasa)/assistants/coffee_shop/models |  |


#### `take_order`

No description provided.

| Argument | Default | Description |
|:-:|:-:|---|
| config | test_take_order |  |
| tablet | true |  |


#### `wait_for_person`

No description provided.

| Argument | Default | Description |
|:-:|:-:|---|
| config | test_wait_for_person |  |


#### `coffee_shop`

No description provided.

| Argument | Default | Description |
|:-:|:-:|---|
| config | full |  |
| tablet | true |  |



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

Response

| Field | Type | Description |
|:-:|:-:|---|
| target_pose_array | geometry_msgs/PoseArray |  |
| target_pointcloud | sensor_msgs/PointCloud2 |  |
| target_point | geometry_msgs/PointStamped |  |

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


### Actions

This package has no actions.
