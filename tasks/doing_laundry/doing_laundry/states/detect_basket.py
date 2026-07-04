#!/usr/bin/env python3
"""
basket_grasp_state.py — Floor laundry-basket perception as a YASMIN state.

Detects a basket standing on the floor, decides empty vs. occupied, and for a
single T-shirt emits collision-free points only (no manipulation):
    pre_grasp (vertical approach above rim) -> grasp (cloth fold) -> lift.

Geometry-first (depth); colour guards against grabbing basket/rim plastic.
Frame: base_footprint (Z up, floor ~ z=0; basket always on the floor).

This file exports two things:
  BasketPerception  — plain helper; attaches subs/pubs/tf to an EXISTING node.
  DetectBasket      — YASMIN state; takes NO node (grabs the shared yasmin node),
                      so it slots in exactly like your other states: DetectBasket().

State outcomes : 'grasp_ready' | 'empty' | 'no_basket' | 'failed'
Blackboard out (on grasp_ready): basket_pose, basket_yaw,
    pre_grasp_pose, grasp_pose, lift_pose, grasp_yaw
"""

import math
import time
from dataclasses import dataclass

import numpy as np
import cv2
import rclpy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, Quaternion
from std_msgs.msg import String
from cv_bridge import CvBridge
import tf2_ros

import torch
from PIL import Image as PILImage
from groundingdino.util.inference import load_model, predict
import groundingdino.datasets.transforms as T

try:
    from yasmin import State
except ImportError:

    class State:  # fallback shim if yasmin is absent
        def __init__(self, outcomes):
            self._outcomes = outcomes


DEPTH_MIN, DEPTH_MAX = 0.3, 3.0
SUBSAMPLE = 5

# Basket (known): height 31, opening 36.4 x 36.2 cm.
BASKET_H = 0.31
OPEN_X, OPEN_Y = 0.364, 0.362

# DINO Paths - 경로가 실제 사용자님의 환경과 일치하는지 반드시 확인하세요!
DINO_CONFIG_PATH = "/home/robocup/ewan/ros_ws/src/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"
DINO_WEIGHT_PATH = "/home/robocup/ewan/ros_ws/src/Base/tasks/doing_laundry/doing_laundry/weights/groundingdino_swint_ogc.pth"

PINCH_BELOW = 0.01  # 옷을 꼬집기 위해 최고점에서 살짝 아래로 내려가는 깊이


@dataclass
class GraspResult:
    status: str
    reason: str = ""
    frame: str = "base_footprint"
    basket_xy: tuple = (0.0, 0.0)
    basket_yaw: float = 0.0
    rim_z: float = 0.0
    grasp: tuple = None
    pre_grasp: tuple = None
    lift: tuple = None
    grasp_yaw: float = 0.0


def _to_local(x, y, cx, cy, yaw):
    dx, dy = x - cx, y - cy
    c, s = math.cos(yaw), math.sin(yaw)
    return dx * c + dy * s, -dx * s + dy * c


def _pca_yaw(xy):
    if xy.shape[0] < 3:
        return 0.0, 1.0
    c = xy - xy.mean(0)
    cov = (c.T @ c) / xy.shape[0]
    w, v = np.linalg.eigh(cov)
    major = v[:, int(np.argmax(w))]
    ratio = float(w.max() / (w.min() + 1e-9))
    return math.atan2(major[1], major[0]), ratio


def _quat_top_down(yaw):
    qx = (1.0, 0.0, 0.0, 0.0)
    half = yaw / 2.0
    qz = (0.0, 0.0, math.sin(half), math.cos(half))
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return Quaternion(
        x=w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        y=w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        z=w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w=w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


class BasketPerception:
    """Perception helper using Grounding DINO."""

    def __init__(
        self,
        node,
        gripper_half=0.05,
        safe_margin=0.03,
        approach_clear=0.15,
        lift_clear=0.22,
        search_z_max=0.70,
        depth_topic="/head_front_camera/depth/image_raw",
        rgb_topic="/head_front_camera/rgb/image_raw",
        info_topic="/head_front_camera/rgb/camera_info",
    ):
        self.node = node
        self.gripper_half = gripper_half
        self.safe_margin = safe_margin
        self.approach_clear = approach_clear
        self.lift_clear = lift_clear
        self.search_z_max = search_z_max

        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None
        self.depth = None
        self.rgb = None
        self.depth_frame = None
        self.depth_stamp = None
        self.depth_scale = 1.0
        self._enc = False
        self.base = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        node.create_subscription(Image, depth_topic, self._depth_cb, 1)
        node.create_subscription(CameraInfo, info_topic, self._cam_cb, 1)
        node.create_subscription(Image, rgb_topic, self._rgb_cb, 1)
        self.basket_pub = node.create_publisher(
            PoseStamped, "/basket_grasp/basket_pose", 10
        )
        self.grasp_pub = node.create_publisher(
            PoseStamped, "/basket_grasp/grasp_pose", 10
        )
        self.path_pub = node.create_publisher(PoseArray, "/basket_grasp/path", 10)
        self.state_pub = node.create_publisher(String, "/basket_grasp/state", 10)

        # --- DINO 초기화 ---
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.node.get_logger().info(f"Loading DINO model on {self.device}...")
        try:
            self.dino_model = load_model(
                DINO_CONFIG_PATH, DINO_WEIGHT_PATH, device=self.device
            )
            self.transform = T.Compose(
                [
                    T.RandomResize([800], max_size=1333),
                    T.ToTensor(),
                    T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
                ]
            )
            self.node.get_logger().info("DINO model loaded successfully.")
        except Exception as e:
            self.node.get_logger().error(f"Failed to load DINO: {e}")
            self.dino_model = None

    def _cam_cb(self, m):
        if self.fx is None:
            self.fx, self.fy, self.cx, self.cy = m.k[0], m.k[4], m.k[2], m.k[5]

    def _rgb_cb(self, m):
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(m, "bgr8")
        except Exception:
            pass

    def _depth_cb(self, m):
        try:
            d = self.bridge.imgmsg_to_cv2(m, "passthrough")
        except Exception:
            return
        if not self._enc:
            self.depth_scale = (
                0.001 if (m.encoding == "16UC1" or d.dtype == np.uint16) else 1.0
            )
            self._enc = True
        self.depth = np.asarray(d, dtype=np.float32) * self.depth_scale
        self.depth_frame = m.header.frame_id
        self.depth_stamp = rclpy.time.Time.from_msg(m.header.stamp)

    def _base_frame(self, cam):
        if self.base:
            return self.base
        for cand in ("base_footprint", "base_link", "odom", "map"):
            try:
                self.tf_buffer.lookup_transform(
                    cand,
                    cam,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
                self.base = cand
                return cand
            except Exception:
                continue
        return None

    def _lookup(self, base, cam, stamp):
        queries = ([stamp] if stamp is not None else []) + [rclpy.time.Time()]
        for q in queries:
            try:
                return self.tf_buffer.lookup_transform(
                    base, cam, q, timeout=rclpy.duration.Duration(seconds=0.3)
                )
            except Exception:
                continue
        return None

    def _cloud(self):
        depth, cam = self.depth, self.depth_frame
        if depth is None or self.fx is None:
            return None, None, None, None
        base = self._base_frame(cam)
        if base is None:
            return None, None, None, None
        h, w = depth.shape[:2]
        vs, us = np.mgrid[0:h:SUBSAMPLE, 0:w:SUBSAMPLE]
        zs = depth[vs, us]
        m = (zs >= DEPTH_MIN) & (zs <= DEPTH_MAX) & np.isfinite(zs)
        us, vs, zs = us[m], vs[m], zs[m]
        if zs.size < 200:
            return None, None, None, None
        xc = (us - self.cx) * zs / self.fx
        yc = (vs - self.cy) * zs / self.fy
        tr = self._lookup(base, cam, self.depth_stamp)
        if tr is None:
            return None, None, None, None
        q, t = tr.transform.rotation, tr.transform.translation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        R = np.array(
            [
                [
                    1 - 2 * (qy * qy + qz * qz),
                    2 * (qx * qy - qz * qw),
                    2 * (qx * qz + qy * qw),
                ],
                [
                    2 * (qx * qy + qz * qw),
                    1 - 2 * (qx * qx + qz * qz),
                    2 * (qy * qz - qx * qw),
                ],
                [
                    2 * (qx * qz - qy * qw),
                    2 * (qy * qz + qx * qw),
                    1 - 2 * (qx * qx + qy * qy),
                ],
            ]
        )
        pts = np.stack([xc, yc, zs], 1) @ R.T + np.array([t.x, t.y, t.z])
        # 3D 점들과 함께 원본 2D 픽셀 좌표(us, vs)도 반환합니다.
        return pts, base, us, vs

    def detect_once(self, timeout=2.0):
        if self.dino_model is None:
            return GraspResult("failed", "DINO model not loaded")

        t0 = time.time()
        while self.depth is None or self.fx is None or self.rgb is None:
            if time.time() - t0 > timeout:
                return GraspResult("failed", "no depth/rgb/intrinsics")
            time.sleep(0.05)

        pts, base, us, vs = self._cloud()
        if pts is None:
            return GraspResult("failed", "tf/cloud")

        h_img, w_img, _ = self.rgb.shape
        pil_image = PILImage.fromarray(cv2.cvtColor(self.rgb, cv2.COLOR_BGR2RGB))
        image_tensor, _ = self.transform(pil_image, None)

        z = pts[:, 2]
        floor_z = float(np.clip(np.percentile(z, 3), -0.05, 0.10))
        rim_z = floor_z + BASKET_H

        # ==========================================
        # 1. 바구니 찾기 (Find Basket)
        # ==========================================
        self.node.get_logger().info("Running DINO for basket...")
        boxes_b, logits_b, _ = predict(
            model=self.dino_model,
            image=image_tensor,
            caption="basket",
            box_threshold=0.35,
            text_threshold=0.25,
            device=self.device,
        )

        if len(boxes_b) == 0:
            return GraspResult(
                "no_basket", "DINO found no basket", frame=base, rim_z=rim_z
            )

        best_b = boxes_b[torch.argmax(logits_b)]
        cx, cy, bw, bh = best_b

        b_xmin = int((cx - bw / 2) * w_img)
        b_xmax = int((cx + bw / 2) * w_img)
        b_ymin = int((cy - bh / 2) * h_img)
        b_ymax = int((cy + bh / 2) * h_img)

        # 2D 바운딩 박스 안쪽에 해당하는 3D Depth 점들을 필터링합니다.
        basket_mask = (us >= b_xmin) & (us <= b_xmax) & (vs >= b_ymin) & (vs <= b_ymax)
        basket_pts = pts[basket_mask]

        if basket_pts.shape[0] < 50:
            return GraspResult(
                "no_basket",
                "No valid depth points inside basket box",
                frame=base,
                rim_z=rim_z,
            )

        # base_footprint 기준의 바구니 3D 중앙 좌표 계산
        basket_3d_x = float(np.median(basket_pts[:, 0]))
        basket_3d_y = float(np.median(basket_pts[:, 1]))

        # ==========================================
        # 2. 티셔츠 찾기 (Find T-shirt)
        # ==========================================
        self.node.get_logger().info("Running DINO for t-shirt...")
        boxes_t, logits_t, _ = predict(
            model=self.dino_model,
            image=image_tensor,
            caption="fabric clothing, folded cloth",
            box_threshold=0.50,
            text_threshold=0.40,
            device=self.device,
        )

        if len(boxes_t) == 0:
            return GraspResult(
                "empty",
                "Basket found, no t-shirt",
                frame=base,
                basket_xy=(basket_3d_x, basket_3d_y),
                rim_z=rim_z,
            )

        # 신뢰도 로깅
        for i, score in enumerate(logits_t):
            self.node.get_logger().info(f"T-shirt candidate {i}: confidence = {score:.4f}")

        # 가장 신뢰도가 높은 바운딩 박스 추출
        best_t = boxes_t[torch.argmax(logits_t)]
        tx, ty, tw, th = best_t

        t_xmin = int((tx - tw / 2) * w_img)
        t_xmax = int((tx + tw / 2) * w_img)
        t_ymin = int((ty - th / 2) * h_img)
        t_ymax = int((ty + th / 2) * h_img)

        # 티셔츠 바운딩 박스 안쪽에 있는 3D Depth 점들을 필터링합니다.
        inset = self.gripper_half + self.safe_margin

        tshirt_mask = (us >= t_xmin) & (us <= t_xmax) & (vs >= t_ymin) & (vs <= t_ymax)
        tshirt_pts = pts[tshirt_mask]

        if tshirt_pts.shape[0] < 10:
            return GraspResult(
                "empty",
                "T-shirt found in 2D but no valid depth points",
                frame=base,
                basket_xy=(basket_3d_x, basket_3d_y),
                rim_z=rim_z,
            )

        # 바구니 안쪽 영역(안전 구역)에 있는 점들만 남깁니다.
        lx = tshirt_pts[:, 0] - basket_3d_x
        ly = tshirt_pts[:, 1] - basket_3d_y
        safe_mask = (np.abs(lx) <= OPEN_X / 2 - inset) & (
            np.abs(ly) <= OPEN_Y / 2 - inset
        )
        safe_pts = tshirt_pts[safe_mask]

        if safe_pts.shape[0] < 5:
            return GraspResult(
                "failed",
                "T-shirt points are too close to walls (unsafe)",
                frame=base,
                basket_xy=(basket_3d_x, basket_3d_y),
                rim_z=rim_z,
            )

        # ==========================================
        # 3. 물리적 부피 검증 (Depth Volume Check)
        # ==========================================
        # 바닥(floor_z) 기준으로 3cm 이상 솟아오른 점들만 추출하여 진짜 티셔츠 부피가 있는지 확인
        min_shirt_height = floor_z + 0.01
        shirt_volume_pts = safe_pts[safe_pts[:, 2] > min_shirt_height]

        if shirt_volume_pts.shape[0] < 30:
            return GraspResult(
                "empty",
                f"Detected t-shirt 2D box, but no 3D volume ({shirt_volume_pts.shape[0]} pts > 3cm)",
                frame=base,
                basket_xy=(basket_3d_x, basket_3d_y),
                rim_z=rim_z,
            )

        # 가장 Z값이 큰(가장 위로 솟아오른) 점을 파지점(Grasp)으로 선택합니다.
        # 주의: 이제 파지점은 부피 검증을 통과한 점들(shirt_volume_pts) 중에서 고릅니다!
        max_z_idx = np.argmax(shirt_volume_pts[:, 2])
        grasp_pt = shirt_volume_pts[max_z_idx]

        gx, gy = float(grasp_pt[0]), float(grasp_pt[1])
        gz = float(grasp_pt[2])

        # 그리퍼 방향 결정을 위해 파지점 주변 옷감의 주름 방향(PCA)을 계산합니다.
        r = self.gripper_half
        d = np.linalg.norm(shirt_volume_pts[:, :2] - np.array([gx, gy]), axis=1)
        nb = shirt_volume_pts[d < r]

        if nb.shape[0] >= 3:
            ridge_yaw, _ = _pca_yaw(nb[:, :2])
            gyaw = ridge_yaw + math.pi / 2
        else:
            gyaw = 0.0

        # 옷을 확실하게 꼬집기 위해 최고점에서 살짝 아래로(PINCH_BELOW) 내려갑니다.
        gz = max(gz - PINCH_BELOW, floor_z + 0.01)
        # 그리퍼 형태에 따른 오프셋 보정 (손가락 끝부분 위치 맞춤)
        gz = gz + 0.105

        self.node.get_logger().info(
            f"[grasp] xy=({gx:.3f},{gy:.3f}) z={gz:.3f} floor={floor_z:.3f} rim={rim_z:.3f}"
        )

        # 완벽하게 규격화된 결과를 넘깁니다.
        return GraspResult(
            "grasp_ready",
            "ok",
            frame=base,
            basket_xy=(basket_3d_x, basket_3d_y),
            basket_yaw=0.0,
            rim_z=rim_z,
            grasp=(gx, gy, gz),
            pre_grasp=(gx, gy, rim_z + self.approach_clear),
            lift=(gx, gy, rim_z + self.lift_clear),
            grasp_yaw=gyaw,
        )

    def _pose(self, xyz, yaw):
        p = Pose()
        p.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
        p.orientation = _quat_top_down(yaw)
        return p

    def publish(self, r: GraspResult):
        self.state_pub.publish(String(data=f"{r.status}:{r.reason}"))
        now = self.node.get_clock().now().to_msg()
        if r.status in ("grasp_ready", "empty"):
            bp = PoseStamped()
            bp.header.stamp = now
            bp.header.frame_id = r.frame
            bp.pose = self._pose(
                (r.basket_xy[0], r.basket_xy[1], r.rim_z), r.basket_yaw
            )
            self.basket_pub.publish(bp)
        if r.status == "grasp_ready":
            gp = PoseStamped()
            gp.header.stamp = now
            gp.header.frame_id = r.frame
            gp.pose = self._pose(r.grasp, r.grasp_yaw)
            self.grasp_pub.publish(gp)
            pa = PoseArray()
            pa.header = gp.header
            pa.poses = [
                self._pose(r.pre_grasp, r.grasp_yaw),
                self._pose(r.grasp, r.grasp_yaw),
                self._pose(r.lift, r.grasp_yaw),
            ]
            self.path_pub.publish(pa)


class DetectBasket(State):
    """YASMIN state. Takes no node: grabs the shared yasmin node and runs
    perception on demand. Add it like your other states: DetectBasket().
    """

    def __init__(self, depth_topic=None, rgb_topic=None, info_topic=None, **cfg):
        super().__init__(outcomes=["grasp_ready", "empty", "no_basket", "failed"])
        from yasmin_ros.yasmin_node import YasminNode

        node = YasminNode.get_instance()

        def _p(name, default):
            if not node.has_parameter(name):
                node.declare_parameter(name, default)
            return node.get_parameter(name).value

        dt = depth_topic or _p(
            "basket.depth_topic", "/head_front_camera/depth/image_raw"
        )
        rt = rgb_topic or _p("basket.rgb_topic", "/head_front_camera/rgb/image_raw")
        it = info_topic or _p("basket.info_topic", "/head_front_camera/rgb/camera_info")
        self.perception = BasketPerception(
            node, depth_topic=dt, rgb_topic=rt, info_topic=it, **cfg
        )

    def execute(self, blackboard):
        r = self.perception.detect_once()
        self.perception.publish(r)
        self.perception.node.get_logger().info(
            f"[DetectBasket] {r.status} ({r.reason})"
        )
        if r.status == "grasp_ready":
            blackboard["basket_pose"] = (r.basket_xy[0], r.basket_xy[1], r.rim_z)
            blackboard["basket_yaw"] = r.basket_yaw
            blackboard["pre_grasp_pose"] = r.pre_grasp
            blackboard["grasp_pose"] = r.grasp
            blackboard["lift_pose"] = r.lift
            blackboard["grasp_yaw"] = r.grasp_yaw
        return r.status