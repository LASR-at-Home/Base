"""Find any named objects with Grounding DINO (lasr_vision_open_vocabulary), optionally checked by a
VLM, with a 3D point for each. Works for objects YOLO/COCO does not know (pringles, rubiks cube...).

Same blackboard output as Detect3D, so it can replace Detect3D in an existing state machine:
    detections_3d : YoloDetection3D.Response; .detected_objects is a list of Detection3D:
                    name (your label), confidence, xywh = [left x, top y, w, h] pixels (as Detect3D),
                    point (target_frame)
                    Ordered: objects with a 3D point first, then by confidence. Objects without a
                    depth reading keep point = (0, 0, 0) unless require_point=True drops them.
    image_raw     : the RGB frame the detections are on
"succeeded" even when nothing is found (like Detect3D): check len(detected_objects).
"failed" only when there is no camera image or the detector does not answer.

What to look for: two lists of the same length, as in a config file.
    prompts: what Grounding DINO is told. Describe what the camera sees, colour + brand + shape
             ("green pringles crisps tube" finds far more than "pringles"); underscores become spaces.
    names:   what comes back in detection.name, exactly as given. Left out: the prompts.

    DetectObjectsOpenVocab(prompts=["red coca cola can", "white paper cup"], names=["coke can", "paper cup"])
    DetectObjectsOpenVocab(prompts=["pringles", "rubiks_cube"])           # names = prompts
    DetectObjectsOpenVocab(config="open_vocab_objects.yaml", vlm=True)    # prompts + names from a file
A config file has the same two lists (example: skills/config/open_vocab_objects.yaml); a bare file
name is looked up in the installed skills/config folder, any other path is read as it is.

vlm=True: a local VLM (Ollama, qwen2.5vl:3b) is asked "is this a <name>?" for each object. On "no"
it picks a name from your list and must confirm it; otherwise the object is dropped
(drop_rejected=False keeps it, at the end). If Ollama does not answer, the VLM step is skipped.
The skill does not move the head: look first (PlayMotion("look_down_centre"), LookToPoint).
Each found frame is drawn on /detect_objects_open_vocab/image (RViz / rqt_image_view).

Needs the detector:
    ros2 run lasr_vision_open_vocabulary open_vocabulary_node --ros-args \\
      -r __node:=lasr_vision_open_vocabulary --params-file \\
      $(ros2 pkg prefix lasr_vision_open_vocabulary)/share/lasr_vision_open_vocabulary/config/params.yaml
Try it alone (default: the objects in skills/config/open_vocab_objects.yaml):
    ros2 run skills detect_objects_open_vocab [--config ~/my_objects.yaml] [--vlm] [--frame map]
"""
import sys
import time
import os
from typing import List, Optional

import numpy as np

import rclpy
import yasmin
from yasmin import Blackboard, StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_ros.yasmin_node import YasminNode

from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time

import tf2_ros
from tf2_geometry_msgs import do_transform_point
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from lasr_vision_interfaces.msg import Detection3D
from lasr_vision_interfaces.srv import OpenVocabDetect, YoloDetection3D

from lasr_skills.vision import open_vocab_utils as ov

DEBUG_TOPIC = "/detect_objects_open_vocab/image"


class _Shared:
    """Camera subscriptions, TF, detector client and debug publisher, made once per process and
    shared by every DetectObjectsOpenVocab (GPSR builds a new state for each command)."""

    _cams = {}
    _common = None

    class Camera:
        def __init__(self, node, rgb, depth, info):
            self.rgb = self.depth = self.info = None
            self.n = 0
            qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
            node.create_subscription(Image, rgb, self._rgb, qos)
            node.create_subscription(Image, depth, self._depth, qos)
            node.create_subscription(CameraInfo, info, self._info, qos)

        def _rgb(self, m):
            self.rgb, self.n = m, self.n + 1

        def _depth(self, m):
            self.depth = m

        def _info(self, m):
            self.info = m

    @classmethod
    def camera(cls, node, rgb, depth, info):
        key = (rgb, depth, info)
        if key not in cls._cams:
            cls._cams[key] = cls.Camera(node, rgb, depth, info)
        return cls._cams[key]

    @classmethod
    def common(cls, node):
        if cls._common is None:
            tf = tf2_ros.Buffer(cache_time=Duration(seconds=30))
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            cls._common = {
                "tf": tf,
                "tf_listener": tf2_ros.TransformListener(tf, node),
                "client": node.create_client(OpenVocabDetect, "/open_vocab/detect"),
                "debug": node.create_publisher(Image, DEBUG_TOPIC, latched),
            }
        return cls._common


def load_objects(config: str):
    """(prompts, names) from a YAML file with the lists `prompts` and `names` (names optional).
    A bare file name is looked up in the installed skills/config folder."""
    import yaml

    path = os.path.expanduser(config)
    if os.sep not in config and not os.path.isfile(path):
        from ament_index_python.packages import get_package_share_directory

        path = os.path.join(get_package_share_directory("skills"), "config", config)
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return list(data.get("prompts") or []), list(data.get("names") or [])


class DetectObjectsOpenVocab(yasmin.State):
    def __init__(
        self,
        prompts: Optional[List[str]] = None,
        names: Optional[List[str]] = None,
        config: Optional[str] = None,    # YAML file with the lists prompts / names, instead of the two above
        box_threshold: float = 0.35,     # tuned on TIAGo in the lab (0.10 glued labels together)
        text_threshold: float = 0.35,
        nms_iou: float = 0.5,
        vlm: bool = False,
        vlm_model: str = ov.DEFAULT_VLM_MODEL,
        vlm_host: str = ov.DEFAULT_VLM_HOST,
        vlm_timeout: float = 30.0,
        drop_rejected: bool = True,
        require_point: bool = False,
        target_frame: str = "map",
        image_topic: str = "/head_front_camera/rgb/image_raw",
        depth_image_topic: str = "/head_front_camera/depth/image_raw",
        camera_info_topic: str = "/head_front_camera/rgb/camera_info",
        timeout: float = 120.0,          # DINO on CPU can take a minute
    ):
        super().__init__(outcomes=["succeeded", "failed"])
        self.set_description("Detects named objects with Grounding DINO (+ optional VLM check), with 3D points")
        self.add_output_key("detections_3d")
        self.add_output_key("image_raw")

        if config:
            prompts, names = load_objects(config)
        prompts = [str(p) for p in (prompts or [])]
        names = [str(n) for n in names] if names else list(prompts)
        if len(names) != len(prompts):
            raise ValueError(f"DetectObjectsOpenVocab: {len(prompts)} prompts but {len(names)} names "
                             f"(they go in pairs, same order)")
        self._labels = {}   # prompt sent to DINO -> name returned
        for prompt, label in zip(prompts, names):
            p = " ".join(str(prompt).replace("_", " ").lower().split())
            if p:
                self._labels.setdefault(p, str(label))
        self._queries = list(self._labels)
        self._box_threshold, self._text_threshold, self._nms_iou = box_threshold, text_threshold, nms_iou
        self._vlm, self._drop_rejected, self._require_point = vlm, drop_rejected, require_point
        self._vlm_kw = {"model": vlm_model, "host": vlm_host, "timeout": vlm_timeout}
        self._frame = target_frame
        self._timeout = timeout

        self._node = YasminNode.get_instance()
        self._bridge = CvBridge()
        self._cam = _Shared.camera(self._node, image_topic, depth_image_topic, camera_info_topic)
        c = _Shared.common(self._node)
        self._tf, self._client, self._debug = c["tf"], c["client"], c["debug"]

    # ---- helpers ----
    def _fresh_frame(self):
        """An RGB frame taken after this state started (the head may have just moved)."""
        n0, t0 = self._cam.n, time.time()
        while time.time() - t0 < 5.0:
            if self._cam.n >= n0 + 2 and self._cam.info is not None:
                return self._cam.rgb
            time.sleep(0.05)
        if self._cam.rgb is not None:
            yasmin.YASMIN_LOG_WARN("detect_objects_open_vocab: no new camera frame for 5 s, using the last one")
        return self._cam.rgb

    def _call(self, req):
        future = self._client.call_async(req)
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > self._timeout:
                return None
            time.sleep(0.05)
        try:
            return future.result()
        except Exception:
            return None

    def _depth_m(self):
        d = self._cam.depth
        if d is None:
            return None
        try:
            img = self._bridge.imgmsg_to_cv2(d, "passthrough").astype(np.float32)
        except Exception:
            return None
        return img / 1000.0 if d.encoding == "16UC1" else img   # the real robot sends millimetres

    def _point(self, rgb, depth_m, box):
        """3D point of the box centre in target_frame, or None."""
        info = self._cam.info
        if depth_m is None or info is None:
            return None
        sx, sy = depth_m.shape[1] / max(1, rgb.width), depth_m.shape[0] / max(1, rgb.height)
        z = ov.box_depth(depth_m, (box[0] * sx, box[1] * sy, box[2] * sx, box[3] * sy))
        if z is None:
            return None
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        ps = PointStamped()
        ps.header = rgb.header
        ps.point.x = (box[0] - cx) * z / fx
        ps.point.y = (box[1] - cy) * z / fy
        ps.point.z = z
        for stamp in (rgb.header.stamp, Time()):   # at the frame's time, else the latest
            try:
                tr = self._tf.lookup_transform(self._frame, rgb.header.frame_id, stamp, timeout=Duration(seconds=0.5))
                return do_transform_point(ps, tr).point
            except Exception:
                continue
        return None

    def _check_with_vlm(self, bgr, dets):
        """[(label, conf, box, rejected)] after the VLM check."""
        out, labels = [], list(dict.fromkeys(self._labels.values()))
        by_lower = {l.lower(): l for l in labels}
        for i, (label, conf, box) in enumerate(dets):
            ok = ov.vlm_is(bgr, box, label.replace("_", " "), **self._vlm_kw)
            if ok is None:
                yasmin.YASMIN_LOG_WARN(f"VLM gave no answer (is Ollama running at {self._vlm_kw['host']} with "
                                       f"{self._vlm_kw['model']}?); keeping the DINO labels")
                return out + [(l, c, b, False) for l, c, b in dets[i:]]
            yasmin.YASMIN_LOG_INFO(f"VLM: is it a {label}? {'yes' if ok else 'no'}")
            if ok:
                out.append((label, conf, box, False))
                continue
            new = None
            if len(labels) > 1:
                pick = ov.vlm_pick(bgr, box, [l.replace("_", " ") for l in labels], **self._vlm_kw)
                pick = by_lower.get(pick) or by_lower.get((pick or "").replace(" ", "_"))
                if pick and pick != label and ov.vlm_is(bgr, box, pick.replace("_", " "), **self._vlm_kw):
                    new = pick
            if new:
                yasmin.YASMIN_LOG_INFO(f"VLM: {label} -> {new} (picked and confirmed)")
                out.append((new, conf, box, False))
            elif not self._drop_rejected:
                out.append((label, conf, box, True))
            else:
                yasmin.YASMIN_LOG_INFO(f"VLM: dropped the {label} at {list(box)}")
        return out

    # ---- state ----
    def execute(self, blackboard: Blackboard) -> str:
        if not self._queries:
            yasmin.YASMIN_LOG_ERROR("DetectObjectsOpenVocab: no objects to look for")
            return "failed"
        rgb = self._fresh_frame()
        if rgb is None:
            yasmin.YASMIN_LOG_ERROR("DetectObjectsOpenVocab: no camera image")
            return "failed"
        if not self._client.wait_for_service(timeout_sec=10.0):
            yasmin.YASMIN_LOG_ERROR("DetectObjectsOpenVocab: /open_vocab/detect is not running")
            return "failed"
        depth_m = self._depth_m()

        req = OpenVocabDetect.Request(image=rgb, queries=self._queries, box_threshold=float(self._box_threshold),
                                      text_threshold=float(self._text_threshold))
        yasmin.YASMIN_LOG_INFO(f"Looking for {self._queries}")
        resp = self._call(req)
        if resp is None:
            yasmin.YASMIN_LOG_ERROR("DetectObjectsOpenVocab: /open_vocab/detect failed or timed out")
            return "failed"

        raw = [(d.name, float(d.confidence), tuple(d.xywh[:4])) for d in resp.detections if len(d.xywh) >= 4]
        named = [(ov.label_for(n, self._queries, self._labels), c, b) for n, c, b in raw]
        dets = ov.nms([d for d in named if d[0]], self._nms_iou)

        bgr = None
        try:
            bgr = self._bridge.imgmsg_to_cv2(rgb, "bgr8")
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"DetectObjectsOpenVocab: cannot convert the image ({e})")
        if self._vlm and bgr is not None and dets:
            checked = self._check_with_vlm(bgr, dets)
        else:
            checked = [(l, c, b, False) for l, c, b in dets]

        found = []
        boxes = {}
        for label, conf, box, rejected in checked:
            cx, cy, w, h = box   # the detector gives the centre; Detect3D gives the top-left corner
            d3 = Detection3D(name=label, confidence=conf, xywh=[int(cx - w / 2), int(cy - h / 2), int(w), int(h)])
            boxes[id(d3)] = box
            pt = self._point(rgb, depth_m, box)
            if pt is not None:
                d3.point = pt
            elif self._require_point:
                yasmin.YASMIN_LOG_INFO(f"no depth for the {label}: dropped (require_point)")
                continue
            found.append((rejected, pt is None, -conf, d3))
        found.sort(key=lambda f: f[:3])
        detections = [f[3] for f in found]

        blackboard["detections_3d"] = YoloDetection3D.Response(detected_objects=detections)
        blackboard["image_raw"] = rgb
        yasmin.YASMIN_LOG_INFO(f"Found {len(detections)} object(s)")
        for (rejected, no_point, _, d) in found:
            where = "no depth" if no_point else f"({d.point.x:.2f}, {d.point.y:.2f}, {d.point.z:.2f}) in {self._frame}"
            yasmin.YASMIN_LOG_INFO(f"  {d.name} {d.confidence:.2f} {where}{'  (VLM: no)' if rejected else ''}")
        if bgr is not None:
            try:
                img = self._bridge.cv2_to_imgmsg(ov.draw(bgr, [(d.name, d.confidence, boxes[id(d)]) for d in detections]),
                                                 encoding="bgr8")
                img.header = rgb.header
                self._debug.publish(img)
            except Exception as e:
                yasmin.YASMIN_LOG_WARN(f"DetectObjectsOpenVocab: debug image not published ({e})")
        return "succeeded"


def main():
    import argparse

    rclpy.init()
    set_ros_loggers()
    ap = argparse.ArgumentParser(description="Look once for the given objects and print what was found.")
    ap.add_argument("--config", default="open_vocab_objects.yaml",
                    help="YAML file with the lists prompts / names (default: the one in skills/config)")
    ap.add_argument("--vlm", action="store_true", help="check each object with the VLM (Ollama)")
    ap.add_argument("--frame", default="map", help="frame for the 3D points (default map)")
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    prompts, names = load_objects(a.config)
    names = names or prompts
    print(f"config: {a.config}")
    for p, n in zip(prompts, names):
        print(f"  {p!r} -> {n!r}")

    sm = StateMachine(outcomes=["succeeded", "failed"])
    sm.add_state("DETECT", DetectObjectsOpenVocab(prompts=prompts, names=names, vlm=a.vlm, target_frame=a.frame),
                 transitions={"succeeded": "succeeded", "failed": "failed"})
    bb = Blackboard()
    outcome = sm(bb)
    print(f"outcome: {outcome}")
    if outcome == "succeeded":
        for d in bb["detections_3d"].detected_objects:
            print(f"  {d.name}  conf {d.confidence:.2f}  box {list(d.xywh)}  "
                  f"point ({d.point.x:.2f}, {d.point.y:.2f}, {d.point.z:.2f})")
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
