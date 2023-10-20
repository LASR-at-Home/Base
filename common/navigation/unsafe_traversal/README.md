# unsafe_traversal

Includes action to move between two points unsafely and service to switch to unsafe
    parameters.

This package is maintained by:
- [Paul Makles](mailto:me@insrt.uk)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- message_generation (build)
- message_runtime (exec)
- rospy
- rosservice
- actionlib
- dynamic_reconfigure
- geometry_msgs
- actionlib_msgs
- nav_msgs



## Usage

Run the node:

```bash
rosrun unsafe_traversal unsafe_traversal
```

> **Warning** Any interactions with this node should be done in a "stop the world" fashion whereby all other processes which have the potential to move the robot must cease in order to minimise the potential of a crash.

> **Warning** ❗ This service does not currently tuck the arm or change the speed of the robot.

### Basic Service Usage

Switch to unsafe mode:

```bash
rosservice call /unsafe_traversal/set_unsafe_traversal true
```

Switch back:

```bash
rosservice call /unsafe_traversal/set_unsafe_traversal false
```

Or use the helper script:

```bash
rosrun unsafe_traversal test_service
```

### Action Usage

This node provides two actions:

- `/unsafe_traversal/move_to_goal` (`unsafe_traversal.msg.MoveToGoalAction`): move to start pose aligned to the end pose then move to the end pose
- `/unsafe_traversal/align_to_goal` (`unsafe_traversal.msg.AlignToGoalAction`): align from start pose to end pose

You can test these by editing `test_move_action` or `test_align_action` in the scripts folder.

### Test Plan Viability

To test whether a plan is viable (can be traversed in a straight line), you can use the viability service.

This provided under the service `/unsafe_traversal/check_if_plan_is_viable`.

You must provide:

```
# starting position
geometry_msgs/PoseStamped start_pose

# end position
geometry_msgs/PoseStamped end_pose
```

And you will get back:

```
# check if viable
bool viable

# the difference between ideal and real plans
float64 raw_error
```

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

#### `DeterminePathViability`

Request

| Field | Type | Description |
|:-:|:-:|---|
| start_pose | geometry_msgs/PoseStamped | starting position |
| end_pose | geometry_msgs/PoseStamped | end position |

Response

| Field | Type | Description |
|:-:|:-:|---|
| viable | bool | check if viable |
| raw_error | float64 | the difference between ideal and real plans |

#### `LaserDist`

Request

| Field | Type | Description |
|:-:|:-:|---|
| fov_degrees | float32 |  |

Response

| Field | Type | Description |
|:-:|:-:|---|
| dist | float32 |  |

#### `ChangeTraversalParameters`

Request

| Field | Type | Description |
|:-:|:-:|---|
| unsafe | bool |  |

Response

| Field | Type | Description |
|:-:|:-:|---|


### Actions

#### `MoveToGoal`

Goal

| Field | Type | Description |
|:-:|:-:|---|
| start_pose | geometry_msgs/PoseStamped | first point to start at |
| end_pose | geometry_msgs/PoseStamped | second point to move to |

Result

| Field | Type | Description |
|:-:|:-:|---|
| success | bool | result |

Feedback

| Field | Type | Description |
|:-:|:-:|---|

#### `AlignToGoal`

Goal

| Field | Type | Description |
|:-:|:-:|---|
| start_pose | geometry_msgs/PoseStamped | first point to position at |
| end_pose | geometry_msgs/PoseStamped | second point to face towards |

Result

| Field | Type | Description |
|:-:|:-:|---|

Feedback

| Field | Type | Description |
|:-:|:-:|---|

