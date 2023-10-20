# lasr_dialogflow

The lasr_dialogflow package

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

#### `DialogflowAudio`

Request

| Field | Type | Description |
|:-:|:-:|---|
| task | std_msgs/String |  |
| action | std_msgs/String |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| result | std_msgs/String |  |
| success | std_msgs/Bool |  |

#### `DialogflowText`

Request

| Field | Type | Description |
|:-:|:-:|---|
| task | std_msgs/String |  |
| action | std_msgs/String |  |
| query_text | std_msgs/String |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| result | std_msgs/String |  |
| success | std_msgs/Bool |  |


### Actions

This package has no actions.
