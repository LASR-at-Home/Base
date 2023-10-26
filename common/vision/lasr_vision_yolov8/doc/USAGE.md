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
