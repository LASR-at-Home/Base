# skills

Skills that can be used in tasks, or to build bigger skills

A skill is a state or a small state machine

This package is maintained by:

- [Maayan Armony](mailto:maayan.armony@gmail.com)

## Prerequisites

This package depends on the following ROS packages:

- colcon (buildtool)
- lasr_vision_interfaces
- lasr_vector_databases_interfaces

This packages requires Python 3.10 to be present.

To use SMACH in ros2, the repository should be cloned and built (done automatically in container):

```shell
# in src dir of the workspace
git clone https://github.com/ros/executive_smach.git
git checkout ros2

cd ../..
colcon build
source install/setup.bash
```