import yasmin
import yasmin_ros

from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from shapely import Polygon as ShapelyPolygon

from lasr_skills import DetectAllInPolygon


class DetectObjects(yasmin.State):
    """
    Looks at a configured surface and detects objects within its polygon
    using YOLO, replacing the open-vocabulary detection approach.

    Uses DetectAllInPolygon (already ported to ROS 2 YASMIN in lasr_skills)
    with a custom or generic YOLO model specified at construction time.

    The polygon is loaded from ROS 2 params.

    Constructor args:
        location_param : str         — config prefix, e.g. "table",
                                       "extra_surface", "breakfast_surface"
        object_filter  : list | None — object class names to filter for.
                                       None detects all known objects from
                                       pick_and_place.objects param.
        model          : str         — YOLO model filename, e.g. "robocup.pt"
                                       or "yolo11n-seg.pt" for generic COCO
        min_confidence : float       — minimum detection confidence

    Blackboard outputs:
        detected_objects : List[Detection3D]
    """

    def __init__(
        self,
        location_param: str = "table",
        object_filter: list = None,
        model: str = "yolo11n-seg.pt",
        min_confidence: float = 0.1,
    ):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_output_key("detected_objects")

        self.node = yasmin_ros.logger_node
        self._model = model
        self._min_confidence = min_confidence

        # Load polygon from config
        try:
            polygon_points = [
                self.node.get_parameter(
                    f"pick_and_place.{location_param}.polygon.top_left"
                ).value,
                self.node.get_parameter(
                    f"pick_and_place.{location_param}.polygon.top_right"
                ).value,
                self.node.get_parameter(
                    f"pick_and_place.{location_param}.polygon.bottom_right"
                ).value,
                self.node.get_parameter(
                    f"pick_and_place.{location_param}.polygon.bottom_left"
                ).value,
            ]
            self._polygon = ShapelyPolygon(polygon_points)
            yasmin.YASMIN_LOG_INFO(
                f"Loaded polygon for '{location_param}': {coords}"
            )
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(
                f"Could not load polygon for '{location_param}': {e}. "
                "Using empty polygon — detections will be unconstrained."
            )
            self._polygon = ShapelyPolygon()

        # Load look point from config
        try:
            lp = self.node.get_parameter(
                f"pick_and_place.{location_param}.look_point"
            ).value
            self._look_point = PointStamped(
                point=Point(x=float(lp[0]), y=float(lp[1]), z=float(lp[2])),
                header=Header(frame_id="map"),
            )
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(
                f"Could not load look_point for '{location_param}': {e}. "
                "Skipping head orientation."
            )
            self._look_point = None

        # ── Load object filter from config or use passed-in list ───────────────
        if object_filter is not None:
            self._object_filter = object_filter
        else:
            try:
                self._object_filter = list(
                    self.node.get_parameter("pick_and_place.objects")
                    .get_parameter_value()
                    .string_array_value
                )
            except Exception:
                yasmin.YASMIN_LOG_WARN(
                    "Could not load object filter from params. "
                    "Detecting all objects."
                )
                self._object_filter = None

    def execute(self, blackboard) -> str:
        # 1. Look at configured point
        if self._look_point is not None:
            # TODO: call LookToPoint with self._look_point
            # from lasr_skills import LookToPoint
            # look = LookToPoint(pointstamped=self._look_point)
            # look.execute(blackboard)
            yasmin.YASMIN_LOG_INFO(
                f"[TODO] LookToPoint at "
                f"({self._look_point.point.x:.2f}, "
                f"{self._look_point.point.y:.2f}, "
                f"{self._look_point.point.z:.2f})"
            )

        # 2. Detect objects within the polygon
        try:
            detector = DetectAllInPolygon(
                polygon=self._polygon,
                object_filter=self._object_filter,
                min_confidence=self._min_confidence,
                model=self._model,
            )

            blackboard["detected_objects"] = []
            blackboard["debug_images"]     = []

            outcome = detector.execute(blackboard)

            if outcome == "failed":
                yasmin.YASMIN_LOG_WARN("DetectAllInPolygon failed.")
                return "failed"

            detected = blackboard["detected_objects"]

            if not detected:
                yasmin.YASMIN_LOG_INFO("No objects detected.")
                return "failed"

            labels = [
                f"{obj.name} ({obj.confidence:.2f})"
                for obj in detected
            ]
            yasmin.YASMIN_LOG_INFO(
                f"Detected {len(detected)} object(s): {', '.join(labels)}"
            )
            return "succeeded"

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Detection failed: {e}")
            return "failed"