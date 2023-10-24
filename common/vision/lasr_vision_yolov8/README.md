# lasr_vision_yolov8

YOLOv8 object detection service

This package is maintained by:
- [Paul Makles](mailto:me@insrt.uk)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- catkin_virtualenv (build)
- lasr_vision_msgs

This packages requires Python 3.10 to be present.

This package has 52 Python dependencies:
- [ultralytics](https://pypi.org/project/ultralytics)==8.0.168
- [dill](https://pypi.org/project/dill)==0.3.7
- .. and 50 sub dependencies



## Usage

This package provides the `/yolov8/detect` service which uses the `YoloDetection` service definition from `lasr_vision_msgs`.

```python
from lasr_vision_msgs.srv import YoloDetection, YoloDetectionRequest

# create service proxy
detect_service = rospy.ServiceProxy('/yolov8/detect', YoloDetection)

# create request
request = YoloDetectionRequest()
request.image_raw = image # sensor_msgs/Image
request.dataset = "yolov8n.pt" # YOLOv8 model, auto-downloads
request.confidence = 0.0 # minimum confidence to include in results
request.nms = 0.0 # non maximal supression

# send request
response = detect_service(request)
# .. use request.detections
```

> You can find [all available (on-demand) models here](https://docs.ultralytics.com/models/yolov8/#supported-tasks).
>
> Place additional models into the `models` folder.

To start the service:

```python
# use the launch file:
roslaunch lasr_vision_yolov8 service.launch
# .. optionally configure debug / preload:
roslaunch lasr_vision_yolov8 service.launch debug:=true preload:=["yolov8n-seg.pt"]
```

## Example

1. Find a video to test on, or otherwise acquire an image topic.

   My test video is `https://www.youtube.com/watch?v=ng8Wivt52K0`, [download it using Cobalt](https://co.wukko.me/) then place it in a directory such as `~/v.mp4`.

2. Then launch the demo:

   ```bash
   roslaunch lasr_vision_yolov8 demo.launch file:=$HOME/v.mp4

   # .. you can also try other models:
   roslaunch lasr_vision_yolov8 demo.launch model:=yolov8n.pt file:=$HOME/v.mp4
   ```

## Technical Overview

There are currently two components to this package:

- The YOLO "server" which runs inference and uses IPC (Python `multiprocessing`) to communicate with a rospy node.
- The actual service node which uses IPC to communicate with the "server".

This is a temporary solution to workaround the minimum Python requirements for the `ultralytics` Python package which wraps around YOLOv8 and provides an interface for running interface and collecting results.

The actual YOLO detection routine works as follows:

- Decode the image using Pillow and numpy

  Pillow is used to decode the image and numpy is used to flip BGR to RGB if necessary.

  The following encodings are currently supported:

  - bgr8
  - 8UC3
  - rgb8

  > [!NOTE]  
  > This could be turned into a utility library.

- Load the appropriate YOLO model

  Models are loaded from the `models` folder. Standard v8 models are loaded on-demand and saved to the directory as well.

  > [!IMPORTANT]  
  > If you would like to train your own model, [a full guide is available here](https://github.com/insertish/yolov8_training_workspace).

  One or more models may be loaded at the same time, they are stored in an in-memory cache.

  > [!WARNING]  
  > Take care to keep track of how many different models you are loading.

- Run inference

  This is entirely handled by the Python package.

- Construct response

  Tensors containing masks are converted to Python arrays to be sent back to the client requesting the detection.

  > [!IMPORTANT]  
  > Tensors may not be stored in CPU memory, so they may have to be moved first using `.cpu()`.

## ROS Definitions

### Launch Files

#### `service`

Start the YOLOv8 service

```bash
# YOLOv8 service
roslaunch lasr_vision_yolov8 service.launch 

# Preload models and enable debug topic
roslaunch lasr_vision_yolov8 service.launch debug:=true preload:=['yolov8n.pt','yolov8n-seg.pt']
```

| Argument | Default | Description |
|:-:|:-:|---|
| debug | false | Whether to publish plotted images to /yolov8/debug |
| preload | [] | Array of models to preload when starting the service |


#### `camera`

Run a YOLOv8 model using the camera

```bash
# Run the demo
roslaunch lasr_vision_yolov8 camera.launch 

# Run the demo with a different model
roslaunch lasr_vision_yolov8 camera.launch model:=yolov8n.pt
```

| Argument | Default | Description |
|:-:|:-:|---|
| model | yolov8n-seg.pt | Model to use for the demo |


#### `demo`

Run a YOLOv8 model on a video file

```bash
# Run the demo
roslaunch lasr_vision_yolov8 demo.launch file:=$HOME/video.mp4

# Run the demo with a different model
roslaunch lasr_vision_yolov8 demo.launch model:=yolov8n.pt file:=$HOME/video.mp4
```

| Argument | Default | Description |
|:-:|:-:|---|
| model | yolov8n-seg.pt | Model to use for the demo |
| file |  | Video file to run inference on |



### Messages

This package has no messages.

### Services

This package has no services.

### Actions

This package has no actions.
