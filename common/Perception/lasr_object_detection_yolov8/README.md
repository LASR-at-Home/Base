# LASR Object Detection Service using YOLOv8

This is an object detection service which uses YOLOv8 as the backend.

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

# send request
response = detect_service(request)
# .. use request.detections
```

> You can find [all available (on-demand) models here](https://docs.ultralytics.com/models/yolov8/#supported-tasks).
>
> Place additional models into the `yolo_server/models` folder.

Before using the service, you must install required packages, see "Installing YOLOv8 Server".

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

## Platform-Specific Notes

When running GUI applications on macOS with Orbstack, ensure the follow environment variables are set:

```bash
# connect to XQuartz over the network
export DISPLAY=<lan ip>:0

# force use of iglx
export LIBGL_ALWAYS_INDIRECT=1
```

To run the YOLO server on Nix, use the shell:

```bash
nix-shell -p python311 python311Packages.numpy python311Packages.pillow python311Packages.opencv4
```

Nvidia driver support works out of the box.

## Installing YOLOv8 Server

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

## Demo Usage

1. Find a video to test on, or otherwise acquire an image topic.

   My test video is `https://www.youtube.com/watch?v=ng8Wivt52K0`, [download it using Cobalt](https://www.youtube.com/watch?v=ng8Wivt52K0) then place it in a directory such as `~/test_video.mp4`.

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
