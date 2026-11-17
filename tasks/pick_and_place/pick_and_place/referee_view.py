"""What the robot sees, for the referee and the video: the camera frame the detector ran on,
every detection boxed with its name and confidence, the object being handled highlighted.

Published on /referee_view twice a second (not once), so a viewer opened at any time shows it:
    ros2 run rqt_image_view rqt_image_view /referee_view
"""
import threading
import time

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

TOPIC = "/referee_view"
PERIOD = 0.5   # s between republishes

GREEN = (0, 200, 0)      # BGR
ORANGE = (0, 165, 255)
WHITE = (255, 255, 255)

_bridge = CvBridge()
_lock = threading.Lock()
_pub = None
_frame = None   # BGR frame the detector ran on
_dets = []      # every Detection3D from that frame (kept after SELECT pops them)
_msg = None     # last drawn Image; _loop republishes it


def _loop():
    while True:
        m = _msg
        if m is not None:
            try:
                _pub.publish(m)
            except Exception:
                pass
        time.sleep(PERIOD)


def _ensure_pub(node):
    global _pub
    with _lock:
        if _pub is None:
            _pub = node.create_publisher(Image, TOPIC, 1)
            threading.Thread(target=_loop, daemon=True).start()


def _label(img, text, x, y, colour):
    font, scale, thick = cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
    (tw, th), base = cv2.getTextSize(text, font, scale, thick)
    y = max(y, th + base + 2)
    x = max(0, min(x, img.shape[1] - tw - 4))   # keep the label inside the picture
    cv2.rectangle(img, (x, y - th - base - 2), (x + tw + 4, y), colour, -1)
    cv2.putText(img, text, (x + 2, y - base), font, scale, (0, 0, 0), thick, cv2.LINE_AA)


def _box(d):
    return tuple(int(v) for v in d.xywh[:4])


def _draw(selected=None, title=""):
    global _msg
    img = _frame.copy()
    sel = None
    for d in _dets:
        x, y, w, h = _box(d)
        if selected is not None and (x, y, w, h) == selected:
            sel = d
            continue
        cv2.rectangle(img, (x, y), (x + w, y + h), GREEN, 2)
        _label(img, f"{d.name} {d.confidence:.2f}", x, y, GREEN)
    if sel is not None:   # drawn last so it sits on top
        x, y, w, h = selected
        cv2.rectangle(img, (x, y), (x + w, y + h), ORANGE, 4)
        _label(img, f"{sel.name} {sel.confidence:.2f}", x, y, ORANGE)
    if title:
        _label(img, title, 5, 30, WHITE)
    _msg = _bridge.cv2_to_imgmsg(img, encoding="bgr8")


def start(node):
    """Create /referee_view at task start, so viewers can pick it before the first detection."""
    _ensure_pub(node)


def show(node, rgb_msg, detections):
    """After detection: the frame with every detection boxed."""
    global _frame, _dets
    _ensure_pub(node)
    _frame = _bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
    _dets = list(detections)
    n = len(_dets)
    _draw(title=f"I can see {n} object{'s' if n != 1 else ''}" if n else "I can see no objects")


def select(node, detection):
    """When an object is picked: the same frame, that object highlighted. False if no frame yet."""
    if _frame is None:
        return False
    _ensure_pub(node)
    _draw(selected=_box(detection), title=f"Now: {detection.name}")
    return True
