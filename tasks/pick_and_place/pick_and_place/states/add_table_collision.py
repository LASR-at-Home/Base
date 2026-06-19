import time
import numpy as np

import yasmin
import yasmin_ros

import rclpy
import rclpy.duration
from rclpy.time import Time as ROS2Time
from rclpy.duration import Duration as ROS2Duration
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PointStamped, Point
from sensor_msgs.msg import Image, CameraInfo

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg

import tf2_ros
from tf2_geometry_msgs import do_transform_point
from cv_bridge import CvBridge

from lasr_vision_interfaces.srv import OpenVocabDetect
from geometry_msgs.msg import Pose, PointStamped, Point, Quaternion


class AddTableCollision(yasmin.State):
    """
    Publishes the table as a box collision object to /collision_object so MoveIt
    plans the arm AROUND it (otherwise it plans through the table and the sim
    physics explodes).

    PRIMARY PATH (detect=True): look down, run open-vocab detection for "table",
    take the largest returned box, grid-sample the depth image inside it, project
    those pixels into MAP, and build the box from the REAL table surface
    (centre x/y, top height z, and — optionally — the seen footprint size).
    This removes the need to hand-tune map coordinates.

    NOTE: CLIP rerank (if enabled for the task) relabels the box to a grocery
    candidate, so we DO NOT trust the detection's name — we take the biggest box.

    FALLBACK PATH (no table detected, or detect=False): use the configured
    base_footprint box, transformed into MAP and locked there.

    Also publishes the detected table centre/size to the blackboard so a later
    ApproachTable state can drive up to it.

    ROS 2 params (pick_and_place.table.collision):
        detect            : bool        - run detection (default True)
        head_tilt         : float       - head tilt while detecting (default -0.4;
                                          use a shallower value e.g. -0.25 to see a
                                          table that is further away)
        frame_id          : str         - fallback input frame (default base_footprint)
        size              : [x, y, z]   - fallback / minimum size
        position          : [x, y, z]   - fallback box centre in frame_id
        use_detected_size : bool        - size box from seen footprint (default True)
        size_margin       : float       - metres added per side of detected size (0.10)

    Blackboard outputs:
        table_point : geometry_msgs/Point  - table centre in map (None if unknown)
        table_size  : [x, y, z]            - table box size used

    Outcomes: succeeded
    """

    HEAD_PAN_JOINT = "head_1_joint"
    HEAD_TILT_JOINT = "head_2_joint"
    HEAD_TILT_DOWN = -0.4

    RGB_TOPIC = "/head_front_camera/rgb/image_raw"
    DEPTH_TOPIC = "/head_front_camera/depth/image_raw"
    INFO_TOPIC = "/head_front_camera/rgb/camera_info"

    DEFAULT_SIZE = [1.2, 1.6, 0.74]
    DEFAULT_POSITION = [1.3, 0.0, 0.37]

    TABLE_QUERY = "box"
    BOX_THRESHOLD = 0.15      # low: tables are big/obvious, keep recall high
    TEXT_THRESHOLD = 0.10
    SURFACE_Z_BAND = 0.08     # m: points within this of the median z are "the top"

    def __init__(self, head_tilt: float = None):
        """
        Args:
            head_tilt: head tilt used while detecting. Overrides the ROS param.
                       Use a shallower value (e.g. -0.25) for the far DETECT_TABLE
                       instance, and the steeper default (-0.4) up close.
        """
        super().__init__(outcomes=["succeeded"])
        self.add_output_key("table_point")
        self.add_output_key("table_size")
        self._head_tilt = head_tilt
        self.node = yasmin_ros.logger_node
        self.bridge = CvBridge()

        self._pub = self.node.create_publisher(CollisionObject, "/collision_object", 10)
        self._tf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf, self.node)

        self._rgb = None
        self._depth = None
        self._info = None
        cam_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.node.create_subscription(Image, self.RGB_TOPIC, self._rgb_cb, cam_qos)
        self.node.create_subscription(Image, self.DEPTH_TOPIC, self._depth_cb, cam_qos)
        self.node.create_subscription(CameraInfo, self.INFO_TOPIC, self._info_cb, cam_qos)

        self._ovd = self.node.create_client(OpenVocabDetect, "open_vocab/detect")
        self._head = ActionClient(
            self.node, FollowJointTrajectory,
            "/head_controller/follow_joint_trajectory",
        )

    # camera callbacks
    def _rgb_cb(self, m):
        self._rgb = m

    def _depth_cb(self, m):
        self._depth = m

    def _info_cb(self, m):
        self._info = m

    # params
    def _param(self, name, default):
        try:
            v = self.node.get_parameter(name).value
            return v if v is not None else default
        except Exception:
            return default
        
    def _quat_param(self, base):
        lst = self._param(base, None)
        if isinstance(lst, (list, tuple)) and len(lst) == 4:
            return [float(v) for v in lst]
        x = self._param(base + ".x", None)
        y = self._param(base + ".y", None)
        z = self._param(base + ".z", None)
        w = self._param(base + ".w", None)
        if None not in (z, w):
            return [float(x or 0.0), float(y or 0.0), float(z), float(w)]
        return None

    def _table_orientation(self):
        o = self._quat_param("pick_and_place.table.collision.orientation")
        if o is None:
            o = self._quat_param("pick_and_place.table.pose.orientation")
        if o is None:
            o = [0.0, 0.0, 0.0, 1.0]
        q = Quaternion()
        q.x, q.y, q.z, q.w = o[0], o[1], o[2], o[3]
        return q
    # head
    def _look_down(self):
        if not self._head.wait_for_server(timeout_sec=5.0):
            yasmin.YASMIN_LOG_WARN("head controller unavailable; skipping look-down")
            return
        tilt = self._head_tilt
        if tilt is None:
            tilt = self._param("pick_and_place.table.collision.head_tilt",
                               self.HEAD_TILT_DOWN)
        tilt = float(tilt)
        pt = JointTrajectoryPoint()
        pt.positions = [0.0, tilt]
        pt.time_from_start = DurationMsg(sec=2)
        traj = JointTrajectory()
        traj.joint_names = [self.HEAD_PAN_JOINT, self.HEAD_TILT_JOINT]
        traj.points = [pt]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self._head.send_goal_async(goal)
        yasmin.YASMIN_LOG_INFO("Tilting head down to look at the table…")
        time.sleep(3.0)

    @staticmethod
    def _wait_future(future, timeout=30.0):
        start = time.time()
        while not future.done():
            if time.time() - start > timeout:
                return None
            time.sleep(0.05)
        try:
            return future.result()
        except Exception:
            return None

    # detection
    def _detect_table_box(self):
        """Return the largest detected box (cx, cy, w, h) for query 'table',
        ignoring the (CLIP-reranked, unreliable) label. None on failure."""
        t0 = time.time()
        while (self._rgb is None or self._info is None) and time.time() - t0 < 5.0:
            time.sleep(0.1)
        if self._rgb is None or self._info is None:
            yasmin.YASMIN_LOG_WARN("No camera image/info — cannot detect table.")
            return None
        if not self._ovd.wait_for_service(timeout_sec=10.0):
            yasmin.YASMIN_LOG_WARN("open_vocab/detect unavailable — cannot detect table.")
            return None

        req = OpenVocabDetect.Request()
        req.image = self._rgb
        req.queries = [self.TABLE_QUERY]
        req.box_threshold = float(self.BOX_THRESHOLD)
        req.text_threshold = float(self.TEXT_THRESHOLD)
        resp = self._wait_future(self._ovd.call_async(req), timeout=120.0)
        if resp is None or not resp.detections:
            yasmin.YASMIN_LOG_WARN("No 'table' detections returned.")
            return None

        best = None
        best_area = -1.0
        for d in resp.detections:
            if len(d.xywh) < 4:
                continue
            cx, cy, w, h = d.xywh[0], d.xywh[1], d.xywh[2], d.xywh[3]
            area = float(w) * float(h)
            if area > best_area:
                best_area = area
                best = (float(cx), float(cy), float(w), float(h))
        if best is not None:
            yasmin.YASMIN_LOG_INFO(
                f"Largest 'table' box cxywh=({best[0]:.0f},{best[1]:.0f},"
                f"{best[2]:.0f},{best[3]:.0f})"
            )
        return best

    def _estimate_table_in_map(self, bbox):
        """Grid-sample depth inside the box, project to MAP, robustly estimate
        the table surface. Returns dict(cx, cy, top_z, sx, sy) or None."""
        if self._depth is None or self._info is None or self._rgb is None:
            return None
        try:
            depth_img = self.bridge.imgmsg_to_cv2(self._depth, "32FC1")
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"depth convert failed: {e}")
            return None

        H, W = depth_img.shape[:2]
        cx, cy, w, h = bbox
        x1, x2 = cx - w / 2.0, cx + w / 2.0
        y1, y2 = cy - h / 2.0, cy + h / 2.0
        # inset 12% so we sample the surface, not the edges/background
        ix, iy = 0.12 * w, 0.12 * h
        gxs = np.linspace(x1 + ix, x2 - ix, 11)
        gys = np.linspace(y1 + iy, y2 - iy, 11)

        K = self._info.k
        fx, fy, cxp, cyp = K[0], K[4], K[2], K[5]
        cam_frame = self._rgb.header.frame_id
        try:
            tr = self._tf.lookup_transform(
                "map", cam_frame, self._rgb.header.stamp,
                timeout=ROS2Duration(seconds=0.5),
            )
        except Exception:
            try:
                tr = self._tf.lookup_transform(
                    "map", cam_frame, ROS2Time(seconds=0),
                    timeout=ROS2Duration(seconds=0.5),
                )
            except Exception as e:
                yasmin.YASMIN_LOG_WARN(f"TF map<-{cam_frame} failed: {e}")
                return None

        pts = []
        for gy in gys:
            py = int(np.clip(gy, 0, H - 1))
            for gx in gxs:
                px = int(np.clip(gx, 0, W - 1))
                d = float(depth_img[py, px])
                if d <= 0.0 or np.isnan(d) or d > 6.0:
                    continue
                ps = PointStamped()
                ps.header.frame_id = cam_frame
                ps.header.stamp = self._rgb.header.stamp
                ps.point.x = (px - cxp) * d / fx
                ps.point.y = (py - cyp) * d / fy
                ps.point.z = d
                try:
                    mp = do_transform_point(ps, tr).point
                except Exception:
                    continue
                pts.append((mp.x, mp.y, mp.z))

        if len(pts) < 12:
            yasmin.YASMIN_LOG_WARN(f"Too few valid depth points ({len(pts)}).")
            return None

        arr = np.array(pts)
        zmed = float(np.median(arr[:, 2]))
        surf = arr[np.abs(arr[:, 2] - zmed) < self.SURFACE_Z_BAND]
        if len(surf) < 8:
            surf = arr
        return {
            "cx": float(np.median(surf[:, 0])),
            "cy": float(np.median(surf[:, 1])),
            "top_z": float(np.median(surf[:, 2])),
            "sx": float(np.percentile(surf[:, 0], 95) - np.percentile(surf[:, 0], 5)),
            "sy": float(np.percentile(surf[:, 1], 95) - np.percentile(surf[:, 1], 5)),
            "n": int(len(surf)),
        }

    # config fallback
    def _lookup_map(self, frame):
        t0 = time.time()
        while time.time() - t0 < 5.0:
            try:
                return self._tf.lookup_transform(
                    "map", frame, ROS2Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0),
                )
            except Exception:
                time.sleep(0.2)
        return None
    def _config_box(self):
        """Configured box, ALWAYS published in 'map' so a hand-set box stays
        locked in place. frame_id from config is intentionally IGNORED — a
        base_footprint box is what made the box follow the robot."""
        size = list(self._param("pick_and_place.table.collision.size", self.DEFAULT_SIZE))
        pos = list(self._param("pick_and_place.table.collision.position", self.DEFAULT_POSITION))
        px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])
        yasmin.YASMIN_LOG_INFO(
            f"Configured table box in 'map' at ({px:.2f}, {py:.2f}, {pz:.2f})."
        )
                
        quat = self._table_orientation()
        return "map", (px, py, pz), [float(size[0]), float(size[1]), float(size[2])], quat
    
    # publish
    def _publish(self, frame, centre, size, orient=None):
        co = CollisionObject()
        co.header.frame_id = frame
        co.id = "table"
        co.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(size[0]), float(size[1]), float(size[2])]
        co.primitives.append(box)

        p = Pose()
        p.position.x, p.position.y, p.position.z = centre
        if orient is not None:
            p.orientation = orient
        else:
            p.orientation.w = 1.0
        co.primitive_poses.append(p)
        co.pose.orientation.w = 1.0
        for _ in range(5):
            self._pub.publish(co)
            time.sleep(0.2)
        yasmin.YASMIN_LOG_INFO(
            f"Published 'table' box centre=({centre[0]:.2f},{centre[1]:.2f},"
            f"{centre[2]:.2f}) size=[{size[0]:.2f},{size[1]:.2f},{size[2]:.2f}] "
            f"frame='{frame}'."
        )


    def _store_table(self, blackboard, cx, cy, cz, size):
        """Expose the table centre/size for a later ApproachTable state."""
        pt = Point()
        pt.x, pt.y, pt.z = float(cx), float(cy), float(cz)
        blackboard["table_point"] = pt
        blackboard["table_size"] = [float(size[0]), float(size[1]), float(size[2])]

    # main
    def execute(self, blackboard) -> str:
        detect = bool(self._param("pick_and_place.table.collision.detect", True))

        if detect:
            self._look_down()
            bbox = self._detect_table_box()
            est = self._estimate_table_in_map(bbox) if bbox is not None else None
            if est is not None:
                cfg_size = list(
                    self._param("pick_and_place.table.collision.size", self.DEFAULT_SIZE)
                )
                use_det = bool(
                    self._param("pick_and_place.table.collision.use_detected_size", True)
                )
                margin = float(
                    self._param("pick_and_place.table.collision.size_margin", 0.10)
                )
                yasmin.YASMIN_LOG_INFO(
                    f"DETECTED table: centre=({est['cx']:.2f},{est['cy']:.2f}) "
                    f"top_z={est['top_z']:.2f} seen_size=({est['sx']:.2f},"
                    f"{est['sy']:.2f}) from {est['n']} pts."
                )
                if use_det:
                    sx = float(np.clip(est["sx"] + 2 * margin, 0.40, 3.0))
                    sy = float(np.clip(est["sy"] + 2 * margin, 0.40, 3.0))
                else:
                    sx, sy = float(cfg_size[0]), float(cfg_size[1])
                top_z = max(0.30, est["top_z"])     # box: floor -> table top
                centre = (est["cx"], est["cy"], top_z / 2.0)
                self._publish("map", centre, [sx, sy, top_z], orient=None)
                # table CENTRE in map (surface height) for ApproachTable
                self._store_table(blackboard, est["cx"], est["cy"], top_z, [sx, sy, top_z])
                return "succeeded"
            yasmin.YASMIN_LOG_WARN("Table detection failed — using configured box.")

        frame, centre, size, orient = self._config_box()
        self._publish(frame, centre, size, orient)
        if frame == "map":
            self._store_table(blackboard, centre[0], centre[1], centre[2], size)
        else:
            blackboard["table_point"] = None
            blackboard["table_size"] = list(size)
        return "succeeded"