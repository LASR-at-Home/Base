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
