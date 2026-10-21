# lasr_vision_yolo

YOLO-based object and pose detection services for ROS 2. Models are loaded once and served on demand, so callers send an image and get detections back.

## Services

| Service | Type | Description |
|---|---|---|
| `/yolo/detect` | `YoloDetection` | 2D detection from an RGB image |
| `/yolo/detect3d` | `YoloDetection3D` | Detection projected to 3D using a depth image and camera info |
| `/yolo/detect_pose` | `YoloPoseDetection` | 2D keypoints |
| `/yolo/detect3d_pose` | `YoloPoseDetection3D` | Keypoints projected to 3D |

Service types are defined in `lasr_vision_interfaces`. 3D results are returned in the request's `target_frame` (default `map`) and need a valid TF from the depth frame.

## Running

```bash
ros2 launch lasr_vision_yolo service_launch.xml
ros2 launch lasr_vision_yolo service_launch.xml preload:="['yolo11n-seg.pt','yolo11n-pose.pt']"
ros2 launch lasr_vision_yolo service_launch.xml sim:=True
```

`demo_launch.xml` and `camera_launch.xml` start the service with an `rqt_image_view` window on `/yolo/debug`.

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `preload` | `['yolo11n-seg.pt']` (launch file) | Models loaded at startup. Other models are loaded on first request |
| `sim` | `False` | Depth units, see below |

### Depth units (`sim`)

The real camera publishes depth as `16UC1` in millimetres, while the simulation publishes it in metres. The service divides depth by 1000 unless `sim` is true:

- `sim:=False` (real robot): depth is converted from mm to metres.
- `sim:=True` (simulation): depth is used as-is.

If you start the service from another launch file, forward the argument to the include, otherwise the default (`False`) applies:

```python
IncludeLaunchDescription(
    AnyLaunchDescriptionSource(path_to_service_launch_xml),
    launch_arguments={"sim": LaunchConfiguration("sim")}.items(),
)
```

## Running from the terminal

Start the service first (see [Running](#running)), then use the `skills` package's test entry points. They grab synced RGB and depth frames from the head camera and call the service:

```bash
ros2 run skills detect_3d                  # YoloDetection3D, logs each detection
ros2 run skills detect_keypoints_3d        # YoloPoseDetection3D
ros2 run skills detect_all_in_polygon      # sweep a hard-coded polygon, detect everything in it
```

You can watch the results in RViz or `rqt_image_view`:

```bash
ros2 run rqt_image_view rqt_image_view /yolo/detect3d/yolo11n_seg_pt   # annotated image
ros2 service list | grep yolo                                           # check the service is up
```

The services take full `sensor_msgs/Image` messages, so calling them with `ros2 service call` by hand isn't practical.

## Calling from Python (skills)

The `skills` package wraps the services as [yasmin](https://github.com/uleroboticsgroup/yasmin) states. They default to `/head_front_camera/rgb/image_raw`, `/head_front_camera/depth/image_raw` and `/head_front_camera/depth/camera_info`.

### `Detect3D`

Calls `/yolo/detect3d`. Writes the response to the blackboard key `detections_3d` (read `.detected_objects`, each with `name`, `confidence`, `xywh`, `xyseg`, `point`) and the RGB frame to `image_raw`.

```python
from lasr_skills import Detect3D

Detect3D(
    model="yolo11n-seg.pt",
    filter=["cup", "bottle"],   # empty/None = all classes
    confidence=0.5,
    target_frame="map",
)
```

### `Detect3DInArea`

`Detect3D` followed by a filter that keeps detections whose `(x, y)` is inside a shapely polygon and whose `z` is inside `[z_min, z_max]`. Leave `area_polygon`, `z_min` or `z_max` out to read them from the blackboard instead (`polygon`, `z_sweep_min`, `z_sweep_max`). After it runs, `detections_3d` is a plain list of `Detection3D`.

```python
from shapely.geometry import Polygon
from lasr_skills import Detect3DInArea

Detect3DInArea(
    area_polygon=Polygon([(0, 0), (2, 0), (2, 2), (0, 2)]),
    z_min=0.3,
    z_max=1.2,
    model="yolo11n-seg.pt",
    filter=["cup"],
)
```

### `DetectKeypoints3D`

Calls `/yolo/detect3d_pose` (default model `yolo11n-pose.pt`) and writes `keypoint_detections_3d` and `image_raw`.

### `DetectAllInPolygon`

Moves the head and base over a polygon and collects every object seen, de-duplicating by `min_new_object_dist`. Takes `polygon`, `model`, `object_filter` and `min_confidence`.

### Using them in a state machine

```python
import rclpy
import yasmin
from yasmin import StateMachine
from yasmin_ros import set_ros_loggers
from lasr_skills import Detect3D

rclpy.init()
set_ros_loggers()

sm = StateMachine(outcomes=["succeeded", "failed"], handle_sigint=True)
sm.add_state(
    "DETECT",
    Detect3D(filter=["person"], target_frame="map"),
    transitions={"succeeded": "succeeded", "failed": "failed"},
)
outcome = sm()
```

Remember to start the YOLO service with `sim:=True` when running in simulation.

## Notes

- The keypoint 3D path assumes a 640x480 depth image.
- Model weights (`*.pt`) are resolved by Ultralytics, so put custom weights where the service can find them.
