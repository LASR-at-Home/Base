import yasmin
import yasmin_ros

from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from shapely import Polygon as ShapelyPolygon

from lasr_skills import DetectAllInPolygon


class DetectCerealMilk(yasmin.State):
    """
    Looks at the cabinet and detects the cereal and milk, which sit next
    to their respective categories per the rulebook setup.

    Standalone state, hardcoded to the cabinet location and a fixed
    object filter, kept separate from ScanShelves which builds the
    general shelf category map rather than searching for two specific
    named items.

    Reads from ROS 2 params:
        cabinet.look_point — x, y, z
        cabinet.polygon    — flat [x0,y0, x1,y1, ...]

    Blackboard outputs:
        detected_objects : List[Detection3D]
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_output_key("detected_objects")

        self.node = yasmin_ros.logger_node

        try:
            self._look_point = PointStamped(
                point=Point(
                    x=self.node.get_parameter("cabinet.look_point.x").value,
                    y=self.node.get_parameter("cabinet.look_point.y").value,
                    z=self.node.get_parameter("cabinet.look_point.z").value,
                ),
                header=Header(frame_id="map"),
            )
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(
                f"Could not load cabinet.look_point from params: {e}. "
                "Using default (0, 0, 0.8)."
            )
            self._look_point = PointStamped(
                point=Point(x=0.0, y=0.0, z=0.8),
                header=Header(frame_id="map"),
            )

        try:
            polygon_flat = self.node.get_parameter("cabinet.polygon").value
            coords = list(zip(polygon_flat[::2], polygon_flat[1::2]))
            self._polygon = ShapelyPolygon(coords)
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(
                f"Could not load cabinet.polygon from params: {e}. "
                "Using empty polygon."
            )
            self._polygon = ShapelyPolygon()

        self._object_filter = ["cereal", "milk"]

    def execute(self, blackboard) -> str:
        # TODO: call LookToPoint with self._look_point once ported to YASMIN
        yasmin.YASMIN_LOG_INFO(
            f"[TODO] Looking at cabinet at point "
            f"({self._look_point.point.x:.2f}, "
            f"{self._look_point.point.y:.2f}, "
            f"{self._look_point.point.z:.2f})."
        )

        try:
            detector = DetectAllInPolygon(
                polygon=self._polygon,
                object_filter=self._object_filter,
                min_confidence=0.1,
                # TODO: switch to robocup.pt or your competition model
                model="yolo11n-seg.pt",
            )

            blackboard["detected_objects"] = []
            blackboard["debug_images"]     = []

            outcome = detector.execute(blackboard)

            if outcome == "failed":
                yasmin.YASMIN_LOG_WARN("DetectAllInPolygon failed for cabinet.")
                return "failed"

            detected = blackboard["detected_objects"]
            labels   = [obj.name for obj in detected]
            yasmin.YASMIN_LOG_INFO(
                f"Detected {len(detected)} object(s) in cabinet: {labels}."
            )

            return "succeeded"

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Cereal/milk detection failed: {e}")
            return "failed"