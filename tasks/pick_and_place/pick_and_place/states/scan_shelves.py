import rclpy
import yasmin
import yasmin_ros

from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from shapely import Polygon as ShapelyPolygon

from lasr_skills import DetectAllInPolygon
from pick_and_place.states.classify_category import ClassifyCategory


class ScanShelves(yasmin.State):
    """
    Iterates over each shelf in the cabinet, detects objects on each shelf,
    and delegates category classification to ClassifyCategory.

    Responsibility of this state: perception only.
        - Read shelf config from params
        - Adjust torso height
        - Look at shelf
        - Detect objects within shelf polygon
        - Store raw object names per shelf

    Category classification is handled by ClassifyCategory (task="shelf").

    Reads from ROS 2 params:
        pick_and_place.cabinet.shelves              — list of shelf IDs
        pick_and_place.cabinet.shelves.<id>.torso_lift_joint
        pick_and_place.cabinet.shelves.<id>.look_point
        pick_and_place.cabinet.shelves.<id>.polygon
        pick_and_place.cabinet.shelves.<id>.z_min
        pick_and_place.cabinet.shelves.<id>.z_max

    Blackboard output:
        shelf_data : dict
            {
                "shelf_1": {
                    "objects":  ["cereal", "oats"],
                    "category": "cereal",
                },
                ...
            }
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_output_key("shelf_data")

        self.node = yasmin_ros.logger_node

        # ClassifyCategory instance reused for each shelf
        self._classifier = ClassifyCategory(task="shelf")

        # TODO: initialise torso action client
        # self._torso_client = ActionClient(
        #     self.node, FollowJointTrajectoryAction,
        #     "/torso_controller/follow_joint_trajectory"
        # )

    def execute(self, blackboard) -> str:
        shelf_data = {}

        # Load shelf IDs from params
        # TODO: confirm param name matches your yaml
        try:
            shelf_ids = (
                self.node.get_parameter("pick_and_place.cabinet.shelves")
                .get_parameter_value()
                .string_array_value
            )
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Could not load shelf IDs from params: {e}")
            return "failed"

        if not shelf_ids:
            yasmin.YASMIN_LOG_ERROR("No shelf IDs found in parameters.")
            return "failed"

        for shelf_id in shelf_ids:
            yasmin.YASMIN_LOG_INFO(f"Scanning shelf: {shelf_id}")

            # ── 1. Get shelf config ───────────────────────────────────────
            if not self._get_shelf_config(shelf_id):
                yasmin.YASMIN_LOG_ERROR(f"Failed to get config for shelf {shelf_id}.")
                return "failed"

            # ── 2. Adjust torso ───────────────────────────────────────────
            self._adjust_torso(shelf_id)

            # ── 3. Look at shelf ──────────────────────────────────────────
            # TODO: call LookToPoint with self._current_look_point
            yasmin.YASMIN_LOG_INFO(
                f"[TODO] Looking at shelf {shelf_id} "
                f"at point {self._current_look_point.point}."
            )

            # ── 4. Detect objects on shelf ────────────────────────────────
            object_names = self._detect_objects(shelf_id, blackboard)

            # ── 5. Classify shelf via ClassifyCategory ────────────────────
            blackboard["object_names"] = object_names
            outcome = self._classifier.execute(blackboard)

            if outcome == "failed":
                yasmin.YASMIN_LOG_WARN(
                    f"ClassifyCategory failed for shelf {shelf_id}. "
                    "Marking as unknown."
                )
                shelf_category = "unknown"
            else:
                shelf_category = blackboard["shelf_category"]

            shelf_data[shelf_id] = {
                "objects":  object_names,
                "category": shelf_category,
            }

            yasmin.YASMIN_LOG_INFO(
                f"Shelf {shelf_id}: category='{shelf_category}', "
                f"objects={object_names}."
            )

        blackboard["shelf_data"] = shelf_data
        return "succeeded"

    # ── Private helpers ───────────────────────────────────────────────────────

    def _get_shelf_config(self, shelf_id: str) -> bool:
        """Reads shelf-specific params and caches them on self."""
        try:
            prefix = f"pick_and_place.cabinet.shelves.{shelf_id}"

            self._current_torso_height = (
                self.node.get_parameter(f"{prefix}.torso_lift_joint")
                .get_parameter_value()
                .double_value
            )

            look_pt = (
                self.node.get_parameter(f"{prefix}.look_point")
                .get_parameter_value()
                .double_array_value
            )
            self._current_look_point = PointStamped(
                point=Point(x=look_pt[0], y=look_pt[1], z=look_pt[2]),
                header=Header(frame_id="map"),
            )

            polygon_flat = (
                self.node.get_parameter(f"{prefix}.polygon")
                .get_parameter_value()
                .double_array_value
            )
            coords = list(zip(polygon_flat[::2], polygon_flat[1::2]))
            self._current_polygon = ShapelyPolygon(coords)

            self._current_z_min = (
                self.node.get_parameter(f"{prefix}.z_min")
                .get_parameter_value()
                .double_value
            )
            self._current_z_max = (
                self.node.get_parameter(f"{prefix}.z_max")
                .get_parameter_value()
                .double_value
            )
            return True

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Error reading params for shelf {shelf_id}: {e}")
            return False

    def _adjust_torso(self, shelf_id: str) -> None:
        """
        Moves the torso to the correct height for viewing this shelf.
        TODO: replace log with FollowJointTrajectory action call.
        """
        yasmin.YASMIN_LOG_INFO(
            f"[TODO] Adjusting torso to {self._current_torso_height:.3f}m "
            f"for shelf {shelf_id}."
        )

    def _detect_objects(self, shelf_id: str, blackboard) -> list:
        """
        Runs DetectAllInPolygon within the shelf polygon and returns
        a list of detected object name strings.

        DetectAllInPolygon outputs to blackboard["detected_objects"] as
        List[Detection3D]. We extract just the names here since that is
        all ClassifyCategory and shelf_data need.
        """
        try:
            detector = DetectAllInPolygon(
                polygon=self._current_polygon,
                min_confidence=0.1,
                # TODO: switch to robocup.pt or your competition model
                model="yolo11n-seg.pt",
            )

            # DetectAllInPolygon needs these keys initialised
            blackboard["detected_objects"] = []
            blackboard["debug_images"]     = []

            outcome = detector.execute(blackboard)

            if outcome == "failed":
                yasmin.YASMIN_LOG_WARN(
                    f"DetectAllInPolygon failed for shelf {shelf_id}."
                )
                return []

            return [obj.name for obj in blackboard["detected_objects"]]

        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"Detection failed for shelf {shelf_id}: {e}")
            return []