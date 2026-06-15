import yasmin
import yasmin_ros

from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from shapely import Polygon as ShapelyPolygon

from lasr_skills import DetectAllInPolygon


class DetectObjects(yasmin.State):
    """
    Looks at the dining table and detects all objects on it within
    a defined polygon.

    Ported from ROS 1 SMACH DetectObjects. The two-state machine
    (LOOK_AT_TABLE → DETECT_OBJECTS) collapses into a single yasmin.State
    since there is no branching between them.

    Uses DetectAllInPolygon (ROS 2 YASMIN version) instead of
    DetectAllInPolygonSensorData — no image is attached to detections.

    Reads from ROS 2 params:
        pick_and_place.table.look_point  — [x, y, z]
        pick_and_place.table.polygon     — flat [x0,y0, x1,y1, ...]
        pick_and_place.objects           — list of object names to filter for

    Blackboard outputs:
        detected_objects : List[Detection3D]
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_output_key("detected_objects")

        self.node = yasmin_ros.get_node()

        # Load params at construction time so errors surface early
        try:
            look_pt_param = (
                self.node.get_parameter("pick_and_place.table.look_point")
                .get_parameter_value()
                .double_array_value
            )
            self._look_point = PointStamped(
                point=Point(
                    x=look_pt_param[0],
                    y=look_pt_param[1],
                    z=look_pt_param[2],
                ),
                header=Header(frame_id="map"),
            )
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(
                f"Could not load table look_point from params: {e}. "
                "Using default (0, 0, 0.8)."
            )
            self._look_point = PointStamped(
                point=Point(x=0.0, y=0.0, z=0.8),
                header=Header(frame_id="map"),
            )

        try:
            polygon_flat = (
                self.node.get_parameter("pick_and_place.table.polygon")
                .get_parameter_value()
                .double_array_value
            )
            coords = list(zip(polygon_flat[::2], polygon_flat[1::2]))
            self._polygon = ShapelyPolygon(coords)
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(
                f"Could not load table polygon from params: {e}. "
                "Using empty polygon."
            )
            self._polygon = ShapelyPolygon()

        try:
            self._object_filter = list(
                self.node.get_parameter("pick_and_place.objects")
                .get_parameter_value()
                .string_array_value
            )
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(
                f"Could not load object filter from params: {e}. "
                "Detecting all objects."
            )
            self._object_filter = None

    def execute(self, blackboard) -> str:
        # ── 1. Look at table ─────────────────────────────────────────────────
        # TODO: call LookToPoint with self._look_point once ported to YASMIN
        yasmin.YASMIN_LOG_INFO(
            f"[TODO] Looking at table at point "
            f"({self._look_point.point.x:.2f}, "
            f"{self._look_point.point.y:.2f}, "
            f"{self._look_point.point.z:.2f})."
        )

        # ── 2. Detect objects within table polygon ───────────────────────────
        try:
            detector = DetectAllInPolygon(
                polygon=self._polygon,
                object_filter=self._object_filter,
                min_confidence=0.1,
                # TODO: switch to robocup.pt or your competition model
                model="yolo11n-seg.pt",
            )

            # DetectAllInPolygon needs these keys initialised
            blackboard["detected_objects"] = []
            blackboard["debug_images"]     = []

            outcome = detector.execute(blackboard)

            if outcome == "failed":
                yasmin.YASMIN_LOG_WARN("DetectAllInPolygon failed.")
                return "failed"

            detected = blackboard["detected_objects"]
            labels   = [obj.name for obj in detected]
            yasmin.YASMIN_LOG_INFO(
                f"Detected {len(detected)} object(s) on table: {labels}."
            )

            return "succeeded"

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Object detection failed: {e}")
            return "failed"