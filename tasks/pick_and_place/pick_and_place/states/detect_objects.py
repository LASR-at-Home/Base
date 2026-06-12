import time

import yasmin
import yasmin_ros

from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from shapely import Polygon as ShapelyPolygon

from lasr_skills import Detect3DInArea

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg
from rclpy.action import ActionClient


class DetectObjects(yasmin.State):
    """
    Tilts the head down to the dining table and detects all objects on it
    within the table polygon — single shot, whole polygon (NO sweep).

    Reads ROS 2 params:
        pick_and_place.table.look_point  — [x, y, z]   (unused now, kept for ref)
        pick_and_place.table.polygon     — flat [x0,y0, x1,y1, ...]
        pick_and_place.objects           — names to keep (empty/missing = all)

    Blackboard outputs:
        detected_objects : List[Detection3D]
    """

    HEAD_PAN_JOINT = "head_1_joint"
    HEAD_TILT_JOINT = "head_2_joint"
    HEAD_TILT_DOWN = -0.3 

    GROCERIES = {
        "cup", "bottle", "bowl", "wine glass", "fork", "knife", "spoon",
        "banana", "apple", "orange", "sandwich", "broccoli", "carrot",
        "hot dog", "pizza", "donut", "cake",
    }

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_output_key("detected_objects")

        self.node = yasmin_ros.logger_node

        # ── look_point (kept for reference / future real point_head) ──
        try:
            lp = (
                self.node.get_parameter("pick_and_place.table.look_point")
                .get_parameter_value()
                .double_array_value
            )
            self._look_point = PointStamped(
                point=Point(x=lp[0], y=lp[1], z=lp[2]),
                header=Header(frame_id="map"),
            )
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"look_point param: {e}; default (0,0,0.8)")
            self._look_point = PointStamped(
                point=Point(x=0.0, y=0.0, z=0.8), header=Header(frame_id="map")
            )

        # ── table polygon ──
        try:
            flat = (
                self.node.get_parameter("pick_and_place.table.polygon")
                .get_parameter_value()
                .double_array_value
            )
            self._polygon = ShapelyPolygon(list(zip(flat[::2], flat[1::2])))
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"polygon param: {e}; empty polygon")
            self._polygon = ShapelyPolygon()

        # ── object filter ──
        try:
            self._object_filter = list(
                self.node.get_parameter("pick_and_place.objects")
                .get_parameter_value()
                .string_array_value
            ) or None
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"objects param: {e}; detecting all")
            self._object_filter = None

        self._head_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            "/head_controller/follow_joint_trajectory",
        )

    # ────────────────────────────────────────────────────────────────────────
    def _look_down(self) -> None:
        """Tilt the head straight down so the camera sees the table."""
        if not self._head_client.wait_for_server(timeout_sec=5.0):
            yasmin.YASMIN_LOG_WARN("head controller unavailable; skipping look-down")
            return
        pt = JointTrajectoryPoint()
        pt.positions = [0.0, self.HEAD_TILT_DOWN]
        pt.time_from_start = DurationMsg(sec=2)
        traj = JointTrajectory()
        traj.joint_names = [self.HEAD_PAN_JOINT, self.HEAD_TILT_JOINT]
        traj.points = [pt]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self._head_client.send_goal_async(goal)
        yasmin.YASMIN_LOG_INFO("Tilting head down to look at the table…")
        time.sleep(3.0)   # let the head settle before detecting

    # ────────────────────────────────────────────────────────────────────────
    def execute(self, blackboard) -> str:
        # 1. look at the table
        self._look_down()

        # 2. detect over the WHOLE table polygon (single shot, no sweep)
        try:
            self._detector = Detect3DInArea(
                area_polygon=self._polygon,
                filter=self._object_filter,
                confidence=0.1,
                z_min=0.3,
                z_max=2.0,
                model="yolo11n-seg.pt",
            )
            outcome = self._detector(blackboard)
            if outcome == "failed":
                yasmin.YASMIN_LOG_WARN("Detect3DInArea failed.")
                return "failed"

            detected = [
                d for d in blackboard["detections_3d"]
                if d.name.lower() in self.GROCERIES
            ]
            blackboard["detected_objects"] = detected
            labels = [obj.name for obj in detected]
            yasmin.YASMIN_LOG_INFO(
                f"Detected {len(detected)} object(s) on table: {labels}."
            )
            return "succeeded" if detected else "failed"

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Object detection failed: {e}")
            time.sleep(1.0)
            return "failed"