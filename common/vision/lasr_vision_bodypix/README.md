# lasr_vision_bodypix

BodyPix 2.0 vision service

This package is maintained by:
- [Paul Makles](mailto:me@insrt.uk)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- catkin_virtualenv (build)

This packages requires Python 3.9 to be present.

This package has 78 Python dependencies:
- [tf-bodypix](https://pypi.org/project/tf-bodypix)==0.4.2
- [tensorflow](https://pypi.org/project/tensorflow)==2.14.0
- [opencv-python](https://pypi.org/project/opencv-python)==4.8.1.78
- [Pillow](https://pypi.org/project/Pillow)==10.1.0
- [matplotlib](https://pypi.org/project/matplotlib)==3.8.1
- .. and 74 sub dependencies

Ask the package maintainer to write or create a blank `doc/PREREQUISITES.md` for their package!

## Usage

Ask the package maintainer to write a `doc/USAGE.md` for their package!

## Example

Ask the package maintainer to write a `doc/EXAMPLE.md` for their package!

## Technical Overview

Ask the package maintainer to write a `doc/TECHNICAL.md` for their package!

## ROS Definitions

### Launch Files

#### `camera`

Run a BodyPix model using the camera

```bash
# Run the demo
roslaunch lasr_vision_bodypix camera.launch 

# Run the demo with a different model
roslaunch lasr_vision_bodypix camera.launch model:=mobilenet50
```

| Argument | Default | Description |
|:-:|:-:|---|
| model | resnet50 | Model to use for the demo |


#### `service`

Start the BodyPix service

```bash
# BodyPix service
roslaunch lasr_vision_bodypix service.launch 

# Preload models and enable debug topic
roslaunch lasr_vision_bodypix service.launch debug:=true preload:=['resnet50', 'mobilenet50']
```

| Argument | Default | Description |
|:-:|:-:|---|
| debug | false | Whether to publish plotted images to /bodypix/debug/model_name |
| preload | [] | Array of models to preload when starting the service |



### Messages

This package has no messages.

### Services

This package has no services.

### Actions

This package has no actions.
