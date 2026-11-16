"""
Follow a person (the host) with the TIAGo using Nav2 + YOLO 3D detections.

ROS2 / yasmin port of the ROS1 `follow_person` state machine. The ROS1 version fused
SAM2, a leg tracker and a Kalman filter; none of those exist in this stack, so tracking
here is "YOLO person detections inside a polygon in front of the robot, associated to the
last known position with a distance gate".

Behaviour (mirrors the ROS1 logic where it still applies):
    * wait for the host to stand in front of the robot, lock onto the closest person
    * continuously re-detect, send Nav2 goals that stop `safe_distance` short of the host
    * if the host is briefly occluded, keep driving to the last known position and wait a
      short grace period before declaring them lost; then scan left/centre/right
    * if the host stops for `stationary_time` seconds (after having walked), ask whether
      we have arrived (yes -> done, no -> re-acquire the host and keep following)
    * bounded retries everywhere so the skill returns "failed" rather than hanging

Blackboard keys owned by this skill (all reset on entry):
    location, cancel_nav, stop_robot_requested, last_known, last_known_stamped,
    detections_3d, polygon, track_memory
"""

import math
import re
import time

import rclpy
from rclpy.time import Time

import yasmin
from yasmin import State, Blackboard, StateMachine, Concurrence
import yasmin_ros
from yasmin_viewer import YasminViewerPub

import tf2_ros

from std_msgs.msg import Header
from geometry_msgs.msg import (
    PointStamped,
    Pose,
    PolygonStamped,
    Point32,
)
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from shapely.geometry import Polygon as ShapelyPolygon

from lasr_skills import (
    Detect3DInArea,
    Wait,
    PlayMotion,
    Say,
    ContinuousGoToLocation,
    WaitForPersonInArea,
    LookToPoint,
    AskAndListen,
)

# Z window (map frame) used for every person detection in this skill.
Z_MIN = -10.0
Z_MAX = 50.0


# -----------------------------------------------
# Helpers
# -----------------------------------------------


def _xy(p):
    """(x, y) from a Point-like object or an (x, y) tuple."""
    if isinstance(p, (tuple, list)):
        return float(p[0]), float(p[1])
    return float(p.x), float(p.y)


def _dist(a, b) -> float:
    ax, ay = _xy(a)
    bx, by = _xy(b)
    return math.hypot(ax - bx, ay - by)


def _bb_get(blackboard, key, default=None):
    """Blackboard read that tolerates a missing key."""
    try:
        if key in blackboard.keys():
            value = blackboard[key]
            return default if value is None else value
    except Exception:
        pass
    return default


class TfRobotPose:
    """Small wrapper around a tf2 buffer to get the robot position in `map`."""

    def __init__(self, node):
        self.node = node
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

    def get(self, timeout: float = 1.0):
        """Returns the robot's translation in map, or None if unavailable."""
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout),
            )
            return transform.transform.translation
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"Waiting for map->base_footprint TF: {e}")
            return None


# -----------------------------------------------
# Small utility states
# -----------------------------------------------


class ResetFollowState(State):
    """
    Resets every blackboard key this skill communicates through, so stale values from a
    previous run / loop (e.g. cancel_nav=True) can't leak into the next one.

    Args:
        clear_target: also forget the tracked person so the next one is acquired fresh.
    """

    def __init__(self, clear_target: bool = True):
        super().__init__(outcomes=["done"])
        self.clear_target = clear_target
        self.add_output_key("location")
        self.add_output_key("cancel_nav")
        self.add_output_key("stop_robot_requested")
        self.add_output_key("last_known")
        self.add_output_key("last_known_stamped")
        self.add_output_key("detections_3d")
        self.add_output_key("track_memory")

    def execute(self, blackboard):
        blackboard["location"] = None
        blackboard["cancel_nav"] = False
        blackboard["stop_robot_requested"] = False
        blackboard["detections_3d"] = []
        blackboard["track_memory"] = {"last_seen": time.monotonic()}
        if self.clear_target:
            blackboard["last_known"] = None
            blackboard["last_known_stamped"] = None
        return "done"


class CancelNavigation(State):
    """Tells ContinuousGoToLocation to stop and exit."""

    def __init__(self):
        super().__init__(outcomes=["done"])
        self.add_output_key("cancel_nav")
        self.add_output_key("stop_robot_requested")

    def execute(self, blackboard):
        blackboard["stop_robot_requested"] = True
        blackboard["cancel_nav"] = True
        return "done"


class ClearDetections(State):
    """Marks the current frame as 'nobody seen' (e.g. after a failed detection call)."""

    def __init__(self):
        super().__init__(outcomes=["done"])
        self.add_output_key("detections_3d")

    def execute(self, blackboard):
        blackboard["detections_3d"] = []
        return "done"


# -----------------------------------------------
# Tracker logic
# -----------------------------------------------


class UpdateDetectionPolygon(State):
    """
    Transforms the base_footprint-relative detection area into a map-frame polygon at the
    robot's current pose.
    Writes: blackboard["polygon"]
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_output_key("polygon")

        self.node = yasmin_ros.logger_node

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        self.base_footprint_polygon = [
            [3.5, 1.25],  # Top Left
            [3.5, -1.25],  # Top Right
            [-0.2, -1.75],  # Bottom Right
            [-0.2, 1.75],  # Bottom Left
        ]

        self.debug_pub = self.node.create_publisher(
            PolygonStamped, "/person_follow/debug/polygon", 10
        )

    def execute(self, blackboard):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )

            debug_polygon = PolygonStamped()
            debug_polygon.header.frame_id = "map"
            debug_polygon.header.stamp = Time().to_msg()

            transformed_polygon = []
            for pt in self.base_footprint_polygon:
                point_stamped = PointStamped()
                point_stamped.header.frame_id = "base_footprint"
                point_stamped.header.stamp = Time().to_msg()
                point_stamped.point.x = pt[0]
                point_stamped.point.y = pt[1]
                point_stamped.point.z = 0.0

                mapped_point = do_transform_point(point_stamped, transform)
                transformed_polygon.append([mapped_point.point.x, mapped_point.point.y])

                debug_polygon.polygon.points.append(
                    Point32(x=mapped_point.point.x, y=mapped_point.point.y, z=0.0)
                )

            blackboard["polygon"] = ShapelyPolygon(transformed_polygon).buffer(0.05)

            self.debug_pub.publish(debug_polygon)
            return "succeeded"
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"TF transform failed: {e}")
            return "failed"


class GetPersonPoint(State):
    """
    Picks which detection is the host and stores it as `last_known`.

    * No previous target: the person closest to the robot.
    * Previous target: the person closest to it. With `gate=True` a detection further than
      `max_jump + walk_speed * seconds_since_seen` from the previous position is rejected
      (so a bystander walking past isn't adopted as the host).

    "failed" means nobody (acceptable) was found; `detections_3d` is then cleared so that
    EvaluateDetections treats it as "person not visible".
    """

    def __init__(
        self,
        gate: bool = True,
        max_jump: float = 1.5,
        walk_speed: float = 1.0,
    ):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("detections_3d")
        self.add_input_key("last_known")
        self.add_input_key("track_memory")

        self.add_output_key("detections_3d")
        self.add_output_key("last_known")
        self.add_output_key("last_known_stamped")
        self.add_output_key("track_memory")

        self.gate = gate
        self.max_jump = max_jump
        self.walk_speed = walk_speed

        self.node = yasmin_ros.logger_node
        self.robot = TfRobotPose(self.node)

    def execute(self, blackboard):
        try:
            detections = [
                d
                for d in _bb_get(blackboard, "detections_3d", [])
                if not math.isnan(d.point.x) and not math.isnan(d.point.y)
            ]
            if not detections:
                blackboard["detections_3d"] = []
                return "failed"

            mem = dict(_bb_get(blackboard, "track_memory", {}))
            now = time.monotonic()
            last_known = _bb_get(blackboard, "last_known")

            if last_known is None:
                reference = self.robot.get(timeout=0.5) or detections[0].point
                best = min(detections, key=lambda d: _dist(d.point, reference))
            else:
                best = min(detections, key=lambda d: _dist(d.point, last_known))
                if self.gate:
                    unseen_for = now - mem.get("last_seen", now)
                    allowed = self.max_jump + self.walk_speed * unseen_for
                    jump = _dist(best.point, last_known)
                    if jump > allowed:
                        yasmin.YASMIN_LOG_WARN(
                            f"Closest person is {jump:.2f}m from the host's last known "
                            f"position (allowed {allowed:.2f}m). Ignoring."
                        )
                        blackboard["detections_3d"] = []
                        return "failed"

            mem["last_seen"] = now
            blackboard["track_memory"] = mem
            blackboard["detections_3d"] = [best]
            blackboard["last_known"] = best.point
            blackboard["last_known_stamped"] = PointStamped(
                header=Header(frame_id="map", stamp=Time().to_msg()),
                point=best.point,
            )
            return "succeeded"
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"GetPersonPoint error: {e}")
            try:
                blackboard["detections_3d"] = []
            except Exception:
                pass
            return "failed"


class EvaluateDetections(State):
    """
    Decides what the robot should do this tick. Run after GetPersonPoint: if
    `detections_3d` is non-empty the host is visible at `last_known`.

    Outcomes:
        updated           - a new Nav2 goal was written to blackboard["location"]
        paused            - robot should hold still (close enough / waiting)
        person_stationary - host has walked, then stopped for `stationary_time`
        person_lost       - host not seen for too long / not found at last known position
    """

    def __init__(
        self,
        safe_distance: float = 1.0,
        move_threshold: float = 0.25,
        stationary_time: float = 6.0,
        min_travel: float = 0.75,
        lost_grace: float = 3.0,
        lost_timeout: float = 10.0,
        close_margin: float = 0.3,
    ):
        super().__init__(
            outcomes=["updated", "paused", "person_stationary", "person_lost"]
        )
        self.add_input_key("detections_3d")
        self.add_input_key("last_known")
        self.add_input_key("track_memory")

        self.add_output_key("location")
        self.add_output_key("cancel_nav")
        self.add_output_key("stop_robot_requested")
        self.add_output_key("track_memory")

        self.safe_distance = safe_distance
        self.move_threshold = move_threshold
        self.stationary_time = stationary_time
        self.min_travel = min_travel
        self.lost_grace = lost_grace
        self.lost_timeout = lost_timeout
        self.close_distance = safe_distance + close_margin

        self.node = yasmin_ros.logger_node
        self.robot = TfRobotPose(self.node)

    def create_goal_pose(self, robot, target) -> Pose:
        """Goal `safe_distance` short of `target`, facing it."""
        rx, ry = _xy(robot)
        tx, ty = _xy(target)
        dx, dy = tx - rx, ty - ry
        distance = math.hypot(dx, dy)

        goal_x, goal_y = tx, ty
        if distance > self.safe_distance:
            goal_x = tx - (dx / distance) * self.safe_distance
            goal_y = ty - (dy / distance) * self.safe_distance

        theta = math.atan2(dy, dx)
        goal = Pose()
        goal.position.x = goal_x
        goal.position.y = goal_y
        goal.orientation.z = math.sin(theta / 2.0)
        goal.orientation.w = math.cos(theta / 2.0)
        return goal

    def execute(self, blackboard: Blackboard):
        robot = self.robot.get(timeout=1.0)
        if robot is None:
            return "paused"

        now = time.monotonic()
        mem = dict(_bb_get(blackboard, "track_memory", {}))
        last_known = _bb_get(blackboard, "last_known")
        detections = _bb_get(blackboard, "detections_3d", [])

        if last_known is None:
            # Nothing to follow yet.
            blackboard["stop_robot_requested"] = True
            return "paused"

        if len(detections) > 0:
            outcome = self._host_visible(blackboard, mem, robot, last_known, now)
        else:
            outcome = self._host_missing(blackboard, mem, robot, last_known, now)

        blackboard["track_memory"] = mem
        return outcome

    def _host_visible(self, blackboard, mem, robot, person, now):
        mem["last_seen"] = now
        px, py = _xy(person)

        # Anchor = where the host last "settled". Comparing against an anchor (not the
        # previous tick) means a slow walker is still recognised as moving.
        anchor = mem.get("anchor")
        if anchor is None or _dist(person, anchor) > self.move_threshold:
            mem["anchor"] = (px, py)
            mem["anchor_time"] = now

        origin = mem.setdefault("origin", (px, py))
        if _dist(person, origin) > self.min_travel:
            mem["has_travelled"] = True

        distance_to_person = _dist(person, robot)
        if distance_to_person <= self.close_distance:
            mem.setdefault("close_since", now)
            blackboard["stop_robot_requested"] = True

            still_for = min(now - mem["anchor_time"], now - mem["close_since"])
            if mem.get("has_travelled") and still_for >= self.stationary_time:
                self.node.get_logger().warn(
                    f"HOST STATIONARY FOR {still_for:.1f}s - STOPPING"
                )
                blackboard["cancel_nav"] = True
                return "person_stationary"
            return "paused"

        mem.pop("close_since", None)
        blackboard["location"] = self.create_goal_pose(robot, person)
        blackboard["stop_robot_requested"] = False
        self.node.get_logger().info("HOST VISIBLE: UPDATING NAV GOAL")
        return "updated"

    def _host_missing(self, blackboard, mem, robot, last_known, now):
        mem.pop("close_since", None)
        unseen_for = now - mem.get("last_seen", now)
        at_last_known = _dist(last_known, robot) <= self.close_distance

        if unseen_for >= self.lost_timeout or (
            at_last_known and unseen_for >= self.lost_grace
        ):
            self.node.get_logger().warn(f"HOST LOST (unseen for {unseen_for:.1f}s)")
            blackboard["stop_robot_requested"] = True
            return "person_lost"

        if at_last_known:
            # Give the host a moment to reappear before starting the recovery scan.
            blackboard["stop_robot_requested"] = True
            return "paused"

        blackboard["location"] = self.create_goal_pose(robot, last_known)
        blackboard["stop_robot_requested"] = False
        self.node.get_logger().warn("HOST NOT VISIBLE: MOVING TO LAST KNOWN POSITION")
        return "updated"


class InitialRecovery(StateMachine):
    """Look centre / left / right for the host. Cancels navigation if not found."""

    class ScanForPerson(StateMachine):
        def __init__(self, direction: str = "centre"):
            super().__init__(outcomes=["succeeded", "failed"])
            self.add_output_key("last_known")

            self.add_state(
                "PLAYMOTION",
                PlayMotion(f"look_{direction}"),
                transitions={
                    "succeeded": "DETECT_3D",
                    "aborted": "failed",
                    "canceled": "failed",
                },
            )
            self.add_state(
                "DETECT_3D",
                Detect3DInArea(filter=["person"], z_min=Z_MIN, z_max=Z_MAX),
                transitions={
                    "succeeded": "GET_PERSON_POINT",
                    "failed": "failed",
                },
            )
            # No gate: the host may have moved a long way while we weren't looking.
            self.add_state(
                "GET_PERSON_POINT",
                GetPersonPoint(gate=False),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_output_key("last_known")
        self.add_output_key("cancel_nav")

        self.add_state(
            "SAY_RECOVERING",
            Say(text="I can't see you."),
            transitions={
                "succeeded": "DETECT_3D_CENTRE",
                "aborted": "DETECT_3D_CENTRE",
                "canceled": "DETECT_3D_CENTRE",
            },
        )
        self.add_state(
            "DETECT_3D_CENTRE",
            self.ScanForPerson("centre"),
            transitions={"succeeded": "succeeded", "failed": "DETECT_3D_LEFT"},
        )
        self.add_state(
            "DETECT_3D_LEFT",
            self.ScanForPerson("left"),
            transitions={"succeeded": "succeeded", "failed": "DETECT_3D_RIGHT"},
        )
        self.add_state(
            "DETECT_3D_RIGHT",
            self.ScanForPerson("right"),
            transitions={"succeeded": "succeeded", "failed": "CANCEL_NAV"},
        )
        self.add_state(
            "CANCEL_NAV",
            CancelNavigation(),
            transitions={"done": "failed"},
        )


class TrackPerson(StateMachine):
    """
    Loop: update polygon -> detect -> associate -> look at host -> evaluate -> wait.
    Runs concurrently with ContinuousGoToLocation, communicating via the blackboard.
    """

    def __init__(self, **evaluate_kwargs):
        super().__init__(outcomes=["person_stationary", "person_lost", "failed"])

        self.add_state(
            "UPDATE_POLYGON",
            UpdateDetectionPolygon(),
            transitions={"succeeded": "DETECT_3D", "failed": "CANCEL_AND_FAIL"},
        )
        self.add_state(
            "DETECT_3D",
            Detect3DInArea(filter=["person"], z_min=Z_MIN, z_max=Z_MAX),
            transitions={
                "succeeded": "GET_PERSON_POINT",
                # A single failed detection call shouldn't end the whole follow;
                # treat it as "not seen this tick" and let the lost-timeouts decide.
                "failed": "CLEAR_DETECTIONS",
            },
        )
        self.add_state(
            "CLEAR_DETECTIONS",
            ClearDetections(),
            transitions={"done": "EVALUATE_DETECTIONS"},
        )
        self.add_state(
            "GET_PERSON_POINT",
            GetPersonPoint(),
            transitions={
                "succeeded": "LOOK_AT_LAST_KNOWN",
                "failed": "EVALUATE_DETECTIONS",
            },
        )
        self.add_state(
            "LOOK_AT_LAST_KNOWN",
            LookToPoint(),
            transitions={
                "succeeded": "EVALUATE_DETECTIONS",
                "aborted": "EVALUATE_DETECTIONS",
                "canceled": "EVALUATE_DETECTIONS",
                "timeout": "EVALUATE_DETECTIONS",
            },
            remappings={"pointstamped": "last_known_stamped"},
        )
        self.add_state(
            "EVALUATE_DETECTIONS",
            EvaluateDetections(**evaluate_kwargs),
            transitions={
                "updated": "WAIT",
                "paused": "WAIT",
                "person_stationary": "person_stationary",
                "person_lost": "BASIC_RECOVERY",
            },
        )
        self.add_state(
            "WAIT",
            Wait(0.2),
            transitions={"succeeded": "UPDATE_POLYGON", "failed": "CANCEL_AND_FAIL"},
        )
        self.add_state(
            "BASIC_RECOVERY",
            InitialRecovery(),
            transitions={"succeeded": "EVALUATE_DETECTIONS", "failed": "person_lost"},
        )
        # Make sure the navigator exits too when the tracker can't continue.
        self.add_state(
            "CANCEL_AND_FAIL",
            CancelNavigation(),
            transitions={"done": "failed"},
        )


# -----------------------------------------------
# Overall following logic
# -----------------------------------------------


class FollowPerson(StateMachine):
    """
    Wait for the host to stand in front of the robot, follow them until they stop and
    confirm (by voice) that we have arrived.

    Outcomes: "succeeded" (arrived), "failed" (could not find / keep the host, or
    repeated lost-person recoveries were exhausted).

    Args:
        safe_distance: how close (m) the robot stops to the host.
        stationary_time: seconds the host must stand still before we ask if we arrived.
        max_lost_attempts: times to ask the host to come back before giving up.
        max_ask_attempts: unanswered/unclear arrival questions before assuming arrival.
    """

    def __init__(
        self,
        safe_distance: float = 1.0,
        stationary_time: float = 6.0,
        max_lost_attempts: int = 3,
        max_ask_attempts: int = 3,
        **evaluate_kwargs,
    ):
        super().__init__(outcomes=["succeeded", "failed"])

        self.max_lost_attempts = max_lost_attempts
        self.max_ask_attempts = max_ask_attempts
        self._lost_attempts = 0
        self._ask_attempts = 0

        self.add_state(
            "INIT",
            yasmin.CbState(outcomes=["done"], callback=self._reset_counters),
            transitions={"done": "RESET_FOLLOW"},
        )
        self.add_state(
            "RESET_FOLLOW",
            ResetFollowState(clear_target=True),
            transitions={"done": "POST_NAV_1"},
        )

        # Start of following
        self.add_state(
            "POST_NAV_1",
            PlayMotion("post_navigation"),
            transitions={
                "succeeded": "UPDATE_POLYGON",
                "aborted": "failed",
                "canceled": "failed",
            },
        )
        self.add_state(
            "UPDATE_POLYGON",
            UpdateDetectionPolygon(),
            transitions={"succeeded": "WAIT_FOR_HOST", "failed": "failed"},
        )
        self.add_state(
            "WAIT_FOR_HOST",
            WaitForPersonInArea(),  # Uses the blackboard polygon
            transitions={
                "succeeded": "GET_PERSON_POINT",
                "failed": "failed",
            },
        )
        self.add_state(
            "GET_PERSON_POINT",
            GetPersonPoint(),
            transitions={"succeeded": "PRE_NAV_1", "failed": "WAIT_FOR_HOST"},
        )
        self.add_state(
            "PRE_NAV_1",
            PlayMotion("pre_navigation"),
            transitions={
                "succeeded": "SAY_FOLLOW",
                "aborted": "failed",
                "canceled": "failed",
            },
        )
        self.add_state(
            "SAY_FOLLOW",
            Say(text="I will now follow you. Lead the way slowly. "),
            transitions={
                "succeeded": "TRACK_AND_NAVIGATE",
                "aborted": "TRACK_AND_NAVIGATE",
                "canceled": "TRACK_AND_NAVIGATE",
            },
        )

        # Tracking and navigating
        self.add_state(
            "TRACK_AND_NAVIGATE",
            Concurrence(
                states={
                    "tracker": TrackPerson(
                        safe_distance=safe_distance,
                        stationary_time=stationary_time,
                        **evaluate_kwargs,
                    ),
                    "navigator": ContinuousGoToLocation(),
                },
                default_outcome="failed",
                outcome_map={
                    "person_stationary": {
                        "tracker": "person_stationary",
                        "navigator": "canceled",
                    },
                    "person_lost": {
                        "tracker": "person_lost",
                        "navigator": "canceled",
                    },
                },
            ),
            transitions={
                "person_stationary": "POST_NAV_2",
                "person_lost": "CALL_LOST_PERSON_BACK",
                "failed": "failed",
            },
        )

        # Lost the host
        self.add_state(
            "CALL_LOST_PERSON_BACK",
            Say(
                text="I seem to have lost track of you. I will wait until you are back in front of me. "
            ),
            transitions={
                "succeeded": "LOST_COUNTER",
                "aborted": "LOST_COUNTER",
                "canceled": "LOST_COUNTER",
            },
        )
        self.add_state(
            "LOST_COUNTER",
            yasmin.CbState(
                outcomes=["retry", "give_up"], callback=self._count_lost_attempt
            ),
            transitions={"retry": "RESET_FOLLOW", "give_up": "failed"},
        )

        # Host stopped: look at them and ask if we have arrived
        self.add_state(
            "POST_NAV_2",
            PlayMotion("post_navigation"),
            transitions={
                "succeeded": "ASK_IF_ARRIVED",
                "aborted": "failed",
                "canceled": "failed",
            },
        )
        self.add_state(
            "ASK_IF_ARRIVED",
            AskAndListen(
                tts_phrase="Say YES if we have arrived. NO if we have not.",
            ),
            transitions={"succeeded": "PROCESS_RESPONSE", "failed": "ASK_RETRY"},
        )
        self.add_state(
            "PROCESS_RESPONSE",
            yasmin.CbState(
                outcomes=["yes", "unknown", "no"],
                callback=self.parse_arrival_confirmation,
            ),
            transitions={
                "yes": "succeeded",
                "unknown": "FEEDBACK_RESPONSE",
                "no": "SAY_CONTINUE",
            },
        )
        self.add_state(
            "FEEDBACK_RESPONSE",
            Say(text="I didn't quite understand that. "),
            transitions={
                "succeeded": "ASK_RETRY",
                "aborted": "ASK_RETRY",
                "canceled": "ASK_RETRY",
            },
        )
        self.add_state(
            "ASK_RETRY",
            yasmin.CbState(
                outcomes=["retry", "give_up"], callback=self._count_ask_attempt
            ),
            # Nobody is answering: assume the destination was reached rather than hang.
            transitions={"retry": "ASK_IF_ARRIVED", "give_up": "succeeded"},
        )
        self.add_state(
            "SAY_CONTINUE",
            Say(text="Okay, please stand in front of me and lead the way. "),
            transitions={
                "succeeded": "RESET_FOLLOW",
                "aborted": "RESET_FOLLOW",
                "canceled": "RESET_FOLLOW",
            },
        )

    # ---- callbacks ----

    def _reset_counters(self, blackboard):
        self._lost_attempts = 0
        self._ask_attempts = 0
        return "done"

    def _count_lost_attempt(self, blackboard):
        self._lost_attempts += 1
        if self._lost_attempts >= self.max_lost_attempts:
            yasmin.YASMIN_LOG_ERROR(
                f"Lost the host {self._lost_attempts} times. Giving up."
            )
            return "give_up"
        return "retry"

    def _count_ask_attempt(self, blackboard):
        self._ask_attempts += 1
        if self._ask_attempts >= self.max_ask_attempts:
            yasmin.YASMIN_LOG_WARN("No usable answer to the arrival question.")
            return "give_up"
        return "retry"

    def parse_arrival_confirmation(self, blackboard):
        response = str(blackboard["transcribed_speech"]).lower()
        yasmin.YASMIN_LOG_INFO(f"Received response: {response}")

        said_yes = re.search(r"\b(yes|yeah|yep|yup|arrived|correct)\b", response)
        said_no = re.search(r"\b(no|nope|not|nah)\b", response)

        if said_yes and not said_no:
            return "yes"
        if said_no and not said_yes:
            self._ask_attempts = 0
            return "no"
        return "unknown"


def main():
    rclpy.init()

    yasmin_ros.set_ros_loggers()

    sm = FollowPerson()
    sm.set_sigint_handler(True)
    bb = Blackboard()

    YasminViewerPub(sm, "Follow_Person")

    outcome = sm(bb)

    yasmin.YASMIN_LOG_INFO(outcome)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
