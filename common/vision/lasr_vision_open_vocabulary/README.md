# open_vocabulary_models

ROS 2 service wrapper for open-vocabulary object detection and segmentation.  
Supports **Grounding DINO** and **YOLOWorld** for detection, and **EfficientViT-SAM** for segmentation.

---
## Configuration

Edit `config/params.yaml` before launching:

| Parameter | Type | Default | Description |
|---|---|---|---|
| `model` | string | `grounding_dino` | Detection model: `grounding_dino` or `yoloworld` |
| `model_device` | string | `cuda` | Device: `cuda` or `cpu` |
| `yoloworld_weights` | string | `yolov8s-world.pt` | YOLOWorld weights file (only if `model: yoloworld`) |
| `use_sam` | bool | `false` | Enable EfficientViT-SAM segmentation |
| `sam_encoder_path` | string | `''` | Path to SAM encoder ONNX model |
| `sam_decoder_path` | string | `''` | Path to SAM decoder ONNX model |

---

## Launch

```bash
colcon build --packages-select lasr_vision_open_vocabulary
source install/setup.bash
ros2 launch lasr_vision_open_vocabulary open_vocab.launch.py
```

---

## Services

### `open_vocab/detect`

Detect objects in an image given a list of text queries.

**Request**
```
sensor_msgs/Image image
string[]          queries
float32           box_threshold
float32           text_threshold
```

**Response**
```
lasr_vision_interfaces/Detection[] detections
```

Each `Detection` has:
- `name` — matched label
- `confidence` — detection score
- `xywh` — bounding box as `[cx, cy, w, h]` in pixels

---

### `open_vocab/detect_and_segment`

Same as `detect`, but also returns segmentation masks via EfficientViT-SAM.  
Requires `use_sam: true` and valid ONNX model paths.

**Request** — same as `detect`

**Response**
```
lasr_vision_interfaces/Detection[] detections
sensor_msgs/Image[]                masks      # mono8, one per detection
int32[]                            ids
```

---

## Testing

Use the `test_detect.py` script. It saves the result image with bounding boxes drawn to `/tmp/open_vocab_result.jpg`.

### From an image file

```bash
python3 common/vision/lasr_vision_open_vocabulary/test_detect.py --image /path/to/image.jpg person cup table
```

### From Tiago camera topic (`/head_front_camera/rgb/image_raw`)

```bash
python3 common/vision/lasr_vision_open_vocabulary/test_detect.py person cup table
```

### With segmentation (requires `use_sam: true` in params)

```bash
# From image file:
python3 common/vision/lasr_vision_open_vocabulary/test_detect.py --image /path/to/image.jpg --segment person cup

# From Tiago camera:
python3 common/vision/lasr_vision_open_vocabulary/test_detect.py --segment person cup
```

### Options

| Argument | Description |
|---|---|
| `queries` (positional) | One or more text queries to detect |
| `--image` | Path to image file. If omitted, uses Tiago camera topic |
| `--output` | Output image path (default: `/tmp/open_vocab_result.jpg`) |
| `--segment` | Use `detect_and_segment` service — overlays masks on result image |
