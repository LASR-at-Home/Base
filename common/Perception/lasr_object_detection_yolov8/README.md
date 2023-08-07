# lasr_object_detection_yolov8

YOLOv8 object detection service

This package is maintained by:
- [Paul Makles](mailto:me@insrt.uk)

## Prerequisites

This package depends on the following ROS packages:
- catkin (buildtool)
- lasr_object_detection_yolo
- lasr_perception_server

Install the YOLOv8 server dependencies:

```bash
# enter source directory
cd src/lasr-base/common/Perception/lasr_object_detection_yolov8/yolo_server

# create a new virtualenv if not already
python3 -m venv venv

# activate it
# bash:
source venv/bin/activate.bash
# fish:
source venv/bin/activate.fish

# install requirements
pip install -r requirements.txt
```

## Usage

This package provides the `/yolov8/detect` service which uses the `YoloDetection` service definition from `lasr_object_detection_yolo`.

```python
from lasr_object_detection_yolo.srv import YoloDetection, YoloDetectionRequest

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
> Place additional models into the `yolo_server/models` folder.

To start the service:

```python
# outside of TIAGo container:
cd src/lasr-base/common/Perception/lasr_object_detection_yolov8/yolo_server
source venv/bin/activate.bash
python3 server.py

# inside of TIAGo container & LASR workspace:
rosrun lasr_object_detection_yolov8 service
# .. or also write to /yolov8/debug topic:
DEBUG=1 rosrun lasr_object_detection_yolov8 service
```

## Example

1. Find a video to test on, or otherwise acquire an image topic.

   My test video is `https://www.youtube.com/watch?v=ng8Wivt52K0`, [download it using Cobalt](https://co.wukko.me/) then place it in a directory such as `~/test_video.mp4`.

2. Start ROS master if not started already.

   ```bash
   roscore &
   ```

3. Start a video stream using the example video. (skip if using another topic / source)

   ```bash
   roslaunch video_stream_opencv camera.launch video_stream_provider:=$HOME/test_video.mp4 loop_videofile:=true visualize:=true
   ```

4. Install the YOLO server if not already, see "Installing YOLOv8 Server" section.

5. Start the YOLO server.

   ```bash
   cd src/lasr-base/common/Perception/lasr_object_detection_yolov8/yolo_server
   source venv/bin/activate.bash
   python3 server.py
   ```

6. Start the YOLO service.

   ```bash
   DEBUG=1 rosrun lasr_object_detection_yolov8 service
   ```

7. Launch image view to preview the debug output.

   ```bash
   rqt_image_view
   ```

8. Start the relay script to start processing images.

   ```bash
   rosrun lasr_object_detection_yolov8 relay /camera/image_raw

   # pick a different model:
   rosrun lasr_object_detection_yolov8 relay /camera/image_raw yolov8n-seg.pt
   rosrun lasr_object_detection_yolov8 relay /camera/image_raw yolov8l.pt

   # example: re-create the mask on the client end:
   rosrun lasr_object_detection_yolov8 construct_mask /camera/image_raw
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

  Models are loaded from the `yolo_server/models` folder. Standard v8 models are loaded on-demand and saved to the directory as well.

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

### Messages

This package has no messages.

### Services

This package has no services.

### Actions

This package has no actions.
