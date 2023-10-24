# LASR Base

This repository contains all of LASR's ROS packages.

Packages are organised into the following folders:

- **common**: packages used throughout the project
  - **helpers**: packages which provide Python / ROS helper functions / services
  - **interfaces**: touch and web interfaces
  - **navigation**: packages for augmenting / controlling navigation
  - **speech**: speech recognition, conversation intent recognition, and text-to-speech
  - **vision**: object recognition
- **documentation** (package): package containing scripts for generating READMEs and also the documentation site itself
- **legacy**: packages intended to be removed / rewritten and adapted into the new structure
- **skills**: commonly used states and small state machines (for smach) which implement various actions
- **tasks**: implementations of robot demos / tasks / challenges

## Documentation

Currently to get the documentation up you should be in our container and then run:

```bash
catkin build documentation
source devel/setup.bash
rosrun documentation view.py
```
