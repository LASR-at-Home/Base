
# Worksheet 2 - YOLOv8

## Introduction
In this worksheet, we will learn how to use YOLOv8. YOLOv8 stands for "You Only Look Once" and is a popular deep learning model for object detection. 

Within LASR, you can find our wrapper for YOLOv8 in `lasr_vision_yolov8`, within the codebase with path : `common/vision/lasr_vision_yolov8/`

It is currently maintained by  [Paul Makles](mailto:me@insrt.uk)

---
### Prerequisites
- Basic knowledge of ROS and Python
- Noetic container
---
## Objectives
By the end of this worksheet, you should be able to:
1. Set up YOLOv8 within ROS.
2. Perform real-time object detection using YOLOv8 using given image or video.
3. Understand how to integrate YOLOv8 within the LASR workspace
---
## Instructions

### Step 1: Choose the models you want to use

> You can find [all available (on-demand) models here](https://docs.ultralytics.com/models/yolov8/#supported-tasks).
>
> Place additional models into the `models` folder.

### Step 2: Take a look at our custom `lasr_vision_msgs`

The `lasr_vision_msgs` are defined msgs to do vision processing within the LASR repo. 
#### `Detection`

| Field | Type | Description |
|:-:|:-:|---|
| name | string | Detected Object Class |
| confidence | float32 | How certain we are this is what we think it is |
| xywh | int32[] | Bounding box mask defined in pixel-space |
| xyseg | int32[] | Segmentation mask defined in pixel-space<br/><br/>This will be empty if a segmentation model was not used |

### Services

#### `YoloDetection`

Request

| Field | Type | Description |
|:-:|:-:|---|
| image_raw | sensor_msgs/Image | Image to run inference on |
| dataset | string | YOLO model to use |
| confidence | float32 | How certain the detection should be to include |
| nms | float32 | Non-maximum Supression<br/><br/>Guiding value to remove overlapping bounding boxes |

Response

| Field | Type | Description |
|:-:|:-:|---|
| detected_objects | lasr_vision_msgs/Detection[] | Detection result |

### Step 3:  Create a srv proxy 

This package provides the `/yolov8/detect` service which uses the `YoloDetection` service definition from `lasr_vision_msgs`.


```python
from lasr_vision_msgs.srv import YoloDetection, YoloDetectionRequest

# create service proxy
detect_service = rospy.ServiceProxy('/yolov8/detect', YoloDetection)

# create request
request = YoloDetectionRequest()
request.image_raw = image # sensor_msgs/Image
request.dataset = "yolov8n.pt" # YOLOv8 model, auto-downloads
request.confidence = 0.5 # minimum confidence to include in results
request.nms = 0.3 # non maximal supression

# send request
response = detect_service(request)
# .. use request.detections
```


### Step 4:  Start the srv

To start the service run this in your container:

```python
# use the launch file:
roslaunch lasr_vision_yolov8 service.launch
# .. optionally configure debug / preload:
roslaunch lasr_vision_yolov8 service.launch debug:=true preload:=["yolov8n-seg.pt"]
```

That is the generic way of calling the service. 

Now try to run an example:

1. Find a video to test on, or otherwise acquire an image topic `sensor_msgs/Image`

   Paul's test video is `https://www.youtube.com/watch?v=ng8Wivt52K0`, [download it using Cobalt](https://co.wukko.me/) then place it in a directory such as `~/v.mp4`.

2. Then launch the demo:

   ```bash
   roslaunch lasr_vision_yolov8 demo.launch file:=$HOME/v.mp4

   # .. you can also try other models:
   roslaunch lasr_vision_yolov8 demo.launch model:=yolov8n.pt file:=$HOME/v.mp4
   ```


---
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
