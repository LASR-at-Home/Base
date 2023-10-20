# train_dataset_model

The train_dataset_model package

This package is maintained by:
- [nicole](mailto:nicole@todo.todo)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- rospy (build)
- std_msgs (build)
- rospy (exec)
- std_msgs (exec)

Ask the package maintainer to write or create a blank `doc/PREREQUISITES.md` for their package!

## Usage

The train model state has the following file :

1. train model => simply call a method to train
2. extract embeddings
3. train model state => i currently use this state to train

on line 19 and line 20 you have to set some variables -> i will work to make it easier in the future

when you have run the create_dataset:

```python
rosrun create_dataset create_dataset_state.py
```

and you have a folder with a random name for instance 'wfeigq'
then you set that folder name in _line 20_ in train_model_state.py
on _line 19_ you set the name that you want to recognise the person with.

I currently take 10 photos and recognise 5 people:

1. flatmate
2. american
3. elisabeth
4. gerard
5. matteo

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

This package has no services.

### Actions

This package has no actions.
