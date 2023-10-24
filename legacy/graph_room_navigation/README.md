# graph_room_navigation

The graph_room_navigation package

This package is maintained by:
- [jared](mailto:jared@todo.todo)

## Prerequisites

This package depends on the following ROS packages:
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

This package has no launch files.

### Messages

This package has no messages.

### Services

#### `AddCrossing`

Request

| Field | Type | Description |
|:-:|:-:|---|
| room1 | string |  |
| door1 | geometry_msgs/Point |  |
| room2 | string |  |
| door2 | geometry_msgs/Point |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| success | bool |  |

#### `AddRoom`

Request

| Field | Type | Description |
|:-:|:-:|---|
| name | string |  |
| a | geometry_msgs/Point |  |
| b | geometry_msgs/Point |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| success | bool |  |

#### `PlanToRoom`

Request

| Field | Type | Description |
|:-:|:-:|---|
| goal_room | string |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| points | geometry_msgs/Point[] |  |
| success | bool |  |

#### `PlanToPoint`

Request

| Field | Type | Description |
|:-:|:-:|---|
| goal | geometry_msgs/Point |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| points | geometry_msgs/Point[] |  |
| success | bool |  |


### Actions

This package has no actions.
