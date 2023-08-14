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
