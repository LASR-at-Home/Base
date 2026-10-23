# lasr_vision_gesture

Gesture detection service. People are classified from 2D pose keypoints, and depth is only used to locate them, so it also works with an RGB-only camera.

It depends on the [`lasr_vision_yolo`](../lasr_vision_yolo/README.md) service being up: `/yolo/detect_pose` for classification and `/yolo/detect3d_pose` for locations (model `yolo11n-pose.pt`).

## Service

| Service | Type |
|---|---|
| `/gesture/detect` | `DetectGesture` (in `lasr_vision_interfaces`) |

Request:

| Field | Default | Description |
|---|---|---|
| `gesture` | | Gesture name, see below |
| `use_3d` | `true` | Return 3D locations. If no depth arrives within 2 s it falls back to 2D |
| `target_frame` | `map` | Frame for 3D locations |
| `timeout` | `5.0` | Seconds to keep looking |

Response: `locations`, one `PointStamped` per person making the gesture. It is empty if nobody did within the timeout. With `use_3d` off (or the 2D fallback) the points are in pixels with `z = 0` and `frame_id` `image`.

## Gestures

Static (decided from a single frame): `raising_left_arm`, `raising_right_arm`, `pointing_left`, `pointing_right`, `lying_down`, `sitting`

Dynamic (needs several frames): `waving`

`sitting` is best effort, as it is weak from a head-on 2D view. Thresholds are constants at the top of `classifiers/static_gestures.py` and `classifiers/dynamic_gestures.py`.

## Running

```bash
ros2 launch lasr_vision_yolo service_launch.xml preload:="['yolo11n-pose.pt']"   # sim:=True in simulation
ros2 launch lasr_vision_gesture gesture_service.launch.xml
```

Camera topics default to the robot's head camera. Override them for another camera, e.g. a webcam:

```bash
ros2 launch lasr_vision_gesture gesture_service.launch.xml rgb_topic:=/image_raw
```

| Argument | Default |
|---|---|
| `rgb_topic` | `/head_front_camera/rgb/image_raw` |
| `depth_topic` | `/head_front_camera/depth/image_raw` |
| `camera_info_topic` | `/head_front_camera/depth/camera_info` |

## Calling it

From the terminal (this service has no images in the request, so it is easy to call by hand):

```bash
ros2 service call /gesture/detect lasr_vision_interfaces/srv/DetectGesture "{gesture: 'waving', timeout: 10.0}"
```

From a state machine, use the `DetectGesture` skill from `skills`. It always uses 3D and can optionally keep only people inside a shapely polygon (otherwise all are returned). Locations go to the blackboard key `gesture_locations`:

```python
from shapely.geometry import Polygon
from lasr_skills import DetectGesture

DetectGesture(
    gesture="waving",
    area_polygon=Polygon([(0, 0), (2, 0), (2, 2), (0, 2)]),  # optional
    target_frame="map",
    timeout=5.0,
)
```
