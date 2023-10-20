# meet_and_greet

The meet_and_greet package

This package is maintained by:
- [nicole](mailto:nicole@todo.todo)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- roscpp (build)
- rospy (build)
- std_msgs (build)
- roscpp (exec)
- rospy (exec)
- std_msgs (exec)

Ask the package maintainer to write or create a blank `doc/PREREQUISITES.md` for their package!

## Usage

> How to run

```python
roslaunch lasr_perception_server perception_server.launch
rosrun meet_and_greet simple_meet_and_greet.py

```

Make sure you have either:

```python
roslaunch custom_worlds wrs_receptionist.launch
roslaunch usb_cam usb_cam-test.launch
```

## Example

Ask the package maintainer to write a `doc/EXAMPLE.md` for their package!

## Technical Overview

> The Meet and greet package files:

1. Explore suroundings state -> robot wondering around (tbc)
2. Look around state -> robot moving the head and searchign for people, to output known and unknown people (tbc)
3. Meet and greet sm -> teh state machine implementing preemtive states (tbc)
4. SImple meet and greet -> the demo file to represent the recognition

## ROS Definitions

### Launch Files

This package has no launch files.

### Messages

This package has no messages.

### Services

This package has no services.

### Actions

This package has no actions.
