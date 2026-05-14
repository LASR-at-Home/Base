# open_vocabulary_models — test scripts

## test_detection.py

Runs a detection on a local image without needing a robot or camera topics.
Requires `open_vocabulary_node` to be running.

**Usage:**
```bash
# Terminal 1 — start the vision node
ros2 run open_vocabulary_models open_vocabulary_node

# Terminal 2 — run the test [objects label]
python3 test_detection.py <image_path> [query1 query2 ...]
```

**Examples:**
```bash
python3 test_detection.py /tmp/scene.jpg bottle cup person
python3 test_detection.py ~/Desktop/test.jpg cocacola chair table
```

**Output:** Opens a window with bounding boxes drawn on the image and prints detections to console.

**Parameters:**
- `image_path` — path to any image file (jpg, png, etc.)
- `queries` — space-separated list of object labels to detect (default: bottle cup person object)

---

## Model configuration

Edit `config/params.yaml` to choose the detection backend:

```yaml
open_vocabulary_models:
  ros__parameters:
    use_grounding_dino: true      # IDEA-Research/grounding-dino-base (best accuracy)
    use_yoloworld: false          # YOLOWorld (faster, less accurate)
    yoloworld_weights: 'yolov8s-world.pt'
    model_device: 'cuda'          # 'cuda' or 'cpu'
    use_sam: false                # EfficientViT-SAM segmentation (requires ONNX models)
    sam_encoder_path: ''
    sam_decoder_path: ''
```

**Grounding DINO** (default):
- Best open-vocabulary accuracy
- Slower (~2s per image on GPU)
- Auto-downloaded from HuggingFace on first run

**YOLOWorld**:
- Faster (~100ms per image on GPU)
- Less accurate on unusual objects
- Requires weights file (e.g. `yolov8s-world.pt`, auto-downloaded by ultralytics)

**Both enabled**: Grounding DINO runs first, YOLOWorld is used as fallback if no detections found.

Override at runtime without editing the file:
```bash
ros2 run open_vocabulary_models open_vocabulary_node --ros-args \
  -p use_grounding_dino:=false \
  -p use_yoloworld:=true \
  -p model_device:=cpu
```
