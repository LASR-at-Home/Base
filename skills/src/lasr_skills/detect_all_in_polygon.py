import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.publisher import Publisher

try:
    from rclpy.executors import EventsExecutor as Executor
except ImportError:
    from rclpy.executors import MultiThreadedExecutor as Executor

import yasmin
import yasmin_ros
from yasmin import Blackboard
from yasmin_viewer import YasminViewerPub

import numpy as np
import cv2

from typing import List, Optional, Tuple

from time import sleep

from threading import Thread, RLock

import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point

from shapely import Polygon as ShapelyPolygon
from shapely import Point as ShapelyPoint
from shapely.affinity import translate
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
from std_msgs.msg import Header
from cv2_img import msg_to_cv2_img, cv2_img_to_msg
from geometry_msgs.msg import (
    Point,
    Point32,
    PointStamped,
    Polygon as ROSPolygon,
    PolygonStamped,
)
from lasr_vision_interfaces.msg import Detection3D

from .look_to_point import LookToPoint
from .detect_3d_in_area import Detect3DInArea

from .wait import Wait


class ProcessDetections(yasmin.State):
    """
    State to process the detected objects and filter them based on the
    minimum distance between objects of the same class.
    """

    _min_new_object_dist: float

    def __init__(self, min_new_object_dist: float = 0.1):
        super().__init__(outcomes=["succeeded", "failed"])

        self.add_input_key("detections_3d")
        self.add_input_key("detected_objects")
        self.add_input_key("image_raw")
        self.add_input_key("debug_images")

        self.add_output_key("detected_objects")
        self.add_output_key("debug_images")

        self._min_new_object_dist = min_new_object_dist

    def execute(self, blackboard) -> str:
        """Processes the detected objects and filters them based on the minimum distance.

        Args:
            userdata (smach.UserData): User data containing the detected objects.

        Returns:
            str: Outcome of the state, "succeeded" or "failed".
        """

        def euclidean_distance(point1: Point, point2: Point) -> float:
            """Calculates the Euclidean distance between two points."""
            return np.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)

        new_detections: List[Detection3D] = []

        try:
            for detection in blackboard["detections_3d"]:
                if detection in blackboard["detected_objects"]:
                    continue

                # Check if the detection is a new object
                is_new_object = True
                for existing_detection in blackboard["detected_objects"]:
                    if (
                        existing_detection.name == detection.name
                        and euclidean_distance(
                            existing_detection.point, detection.point
                        )
                        < self._min_new_object_dist
                    ):
                        yasmin_ros.logger_node.get_logger().info(
                            f"Detected object {detection.name} is too close to existing object {existing_detection.name}. Not counting as new."
                        )
                        is_new_object = False
                        break

                if is_new_object:
                    new_detections.append(detection)

            blackboard["debug_images"].append((blackboard["image_raw"], new_detections))
            blackboard["detected_objects"].extend(new_detections)
            yasmin_ros.logger_node.get_logger().info(
                f"Processed detections. Total detected objects: {len(blackboard['detected_objects'])}"
            )
            yasmin_ros.logger_node.get_logger().info("Detected objects:")
            for obj in blackboard["detected_objects"]:
                yasmin_ros.logger_node.get_logger().info(
                    f" - {obj.name} at ({obj.point.x}, {obj.point.y}, {obj.point.z})"
                )
            return "succeeded"
        except Exception as e:
            yasmin_ros.logger_node.get_logger().error(
                f"Failed to process detections: {e}"
            )
            return "failed"


class CalculateSweepPoints(yasmin.State):
    """
    State to calculate the points to sweep based on the polygon and camera FOV.
    """

    def __init__(
        self,
        polygon: ShapelyPolygon,
        min_coverage: float = 0.8,
        z_axis: float = 0.7,
        fov_depth: float = 2.0,
    ):
        super().__init__(outcomes=["succeeded", "failed"])

        self.add_output_key("sweep_points")
        self.add_output_key("sweep_point_index")

        self._polygon = polygon
        self._min_coverage = min_coverage
        self._z_axis = z_axis
        self._fov_depth = fov_depth

        self.node = yasmin_ros.logger_node

        self._tf_buffer = tf2_ros.Buffer(Duration(seconds=10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self.node)

    def _get_camera_fov_polygon(self) -> ShapelyPolygon:
        """
        Projects the camera's FOV to the ground plane using intrinsics and TF.

        Returns:
            ShapelyPolygon: Footprint of camera FOV in map frame.
        """

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
        )

        success, msg = rclpy.wait_for_message.wait_for_message(
            msg_type=CameraInfo,
            node=self.node,
            topic="/head_front_camera/depth/camera_info",
            qos_profile=qos,
            time_to_wait=5,
        )

        if success is False:
            yasmin.YASMIN_LOG_INFO("No camera info received, ending state")
            self.cancel_state()

        model = PinholeCameraModel()
        model.fromCameraInfo(msg)

        # Define pixel corners (image boundaries)
        corners = [
            (0, 0),  # top-left
            (model.width - 1, 0),  # top-right
            (model.width - 1, model.height - 1),  # bottom-right
            (0, model.height - 1),  # bottom-left
        ]

        qos_test = QoSProfile(history=HistoryPolicy.KEEP_ALL)

        pub = self.node.create_publisher(PointStamped, "fov_corners", qos_test)

        # Transform pixel rays to map frame
        transformed_points = []
        for u, v in corners:
            ray = model.projectPixelTo3dRay((u, v))
            point_cam = PointStamped()
            point_cam.header.frame_id = msg.header.frame_id
            point_cam.header.stamp = Time().to_msg()
            point_cam.point.x = ray[0] * self._fov_depth
            point_cam.point.y = ray[1] * self._fov_depth
            point_cam.point.z = ray[2] * self._fov_depth

            pub.publish(point_cam)

            # Transform to map frame
            try:
                transform = self._tf_buffer.lookup_transform(
                    "map",
                    msg.header.frame_id,
                    Time(),
                    timeout=Duration(seconds=5.0),
                )
                point_map = do_transform_point(point_cam, transform)
                transformed_points.append((point_map.point.x, point_map.point.y))
            except Exception as e:
                yasmin.YASMIN_LOG_ERROR(
                    f"Transform failed with camera timestamp: {e}. Retrying with latest TF data."
                )

        return ShapelyPolygon(transformed_points)

    def _extract_relative_footprint(
        self, camera_hull: ShapelyPolygon
    ) -> ShapelyPolygon:
        """Shift camera hull so its centroid is at (0, 0)"""
        centroid = camera_hull.centroid
        return translate(camera_hull, xoff=-centroid.x, yoff=-centroid.y)

    def _place_footprint_at_point(
        self, rel_hull: ShapelyPolygon, look_point: ShapelyPoint
    ) -> ShapelyPolygon:
        """Translate relative hull to a new centroid position"""
        return translate(rel_hull, xoff=look_point.x, yoff=look_point.y)

    def _sample_points_in_polygon(
        self, polygon: ShapelyPolygon, num_samples: int = 1000
    ) -> List[ShapelyPoint]:
        """Randomly samples points within a polygon."""
        minx, miny, maxx, maxy = polygon.bounds
        samples = []
        while len(samples) < num_samples:
            p = ShapelyPoint(
                np.random.uniform(minx, maxx),
                np.random.uniform(miny, maxy),
            )
            if polygon.contains(p):
                samples.append(p)
        return samples

    def _greedy_coverage_min_overlap(
        self,
        candidate_footprints: List[ShapelyPolygon],
        coverage_goal: float = 0.9,
        overlap_penalty: float = 0.5,
    ) -> Tuple[List[ShapelyPolygon], ShapelyPolygon]:
        covered = ShapelyPolygon()
        selected = []
        total_area = self._polygon.area
        remaining = candidate_footprints.copy()

        while covered.area / total_area < coverage_goal and remaining:
            best_score = -np.inf
            best_fp = None
            best_intersection = None

            for fp in remaining:
                intersection = fp.intersection(self._polygon)
                new_area = intersection.difference(covered).area
                overlap_area = intersection.intersection(covered).area
                score = new_area - overlap_penalty * overlap_area

                if score > best_score:
                    best_score = score
                    best_fp = fp
                    best_intersection = intersection

            if best_score <= 0:
                break

            selected.append(best_fp)
            covered = covered.union(best_intersection)
            remaining.remove(best_fp)

            yasmin.YASMIN_LOG_INFO(
                f"Selected new footprint, total coverage: {covered.area / total_area:.2%}, score: {best_score:.2f}"
            )

        return selected, covered

    def _calculate_sweep_points(self) -> List[PointStamped]:
        """
        Calculates the points to sweep based on the polygon and FOV projection.

        Returns:
            List[PointStamped]: List of sweep points in map frame.
        """
        yasmin.YASMIN_LOG_INFO("Waiting for camera info and TF to map frame...")
        fov_polygon = self._get_camera_fov_polygon()

        # Optional: visualize FOV

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        pub = self.node.create_publisher(PolygonStamped, "projected_fov_polygon", qos)

        pub.publish(
            PolygonStamped(
                header=Header(
                    frame_id="map", stamp=self.node.get_clock().now().to_msg()
                ),
                polygon=ROSPolygon(
                    points=[
                        Point32(x=x, y=y, z=0.0) for x, y in fov_polygon.exterior.coords
                    ]
                ),
            )
        )

        rel_camera_hull = self._extract_relative_footprint(fov_polygon)
        sampled_points = self._sample_points_in_polygon(self._polygon, num_samples=10)
        candidate_footprints = [
            self._place_footprint_at_point(rel_camera_hull, p) for p in sampled_points
        ]

        selected_footprints, _ = self._greedy_coverage_min_overlap(
            candidate_footprints,
            coverage_goal=self._min_coverage,
            overlap_penalty=0.0,
        )

        sweep_points = [
            PointStamped(
                header=Header(frame_id="map"),
                point=Point(x=fp.centroid.x, y=fp.centroid.y, z=self._z_axis),
            )
            for fp in selected_footprints
        ]

        qos_test = QoSProfile(history=HistoryPolicy.KEEP_ALL)

        point_pub = self.node.create_publisher(PointStamped, "/sweep_points", qos_test)

        for point in sweep_points:
            point_pub.publish(point)

        yasmin.YASMIN_LOG_INFO(f"Calculated {len(sweep_points)} sweep points.")
        return sweep_points

    def execute(self, blackboard) -> str:
        """Main execution entrypoint."""
        try:
            blackboard["sweep_points"] = self._calculate_sweep_points()
            blackboard["sweep_point_index"] = 0
            return "succeeded"
        except Exception as e:
            yasmin_ros.logger_node.get_logger().error(
                f"Failed to calculate sweep points: {e}"
            )
            return "failed"


class IterateThroughPoints(yasmin.StateMachine):
    def __init__(
        self,
        polygon: ShapelyPolygon,
        model: str = "yolo11n-seg.pt",
        object_filter: Optional[List[str]] = None,
        min_confidence: float = 0.5,
        min_new_object_dist: float = 0.1,
    ):
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)

        get_point_state = yasmin.CbState(
            outcomes=["succeeded", "failed", "continue"], callback=self._get_look_point
        )
        get_point_state.add_input_key("sweep_points")
        get_point_state.add_input_key("sweep_point_index")
        get_point_state.add_output_key("pointstamped")
        get_point_state.add_output_key("sweep_point_index")

        self.add_state(
            "GET_LOOK_POINT",
            get_point_state,
            transitions={
                "succeeded": "succeeded",
                "continue": "LOOK_POINT",
                "failed": "failed",
            },
        )
        self.add_state(
            "LOOK_POINT",
            LookToPoint(),
            transitions={
                "succeeded": "DETECT_OBJECTS",
                "aborted": "DETECT_OBJECTS",
                "canceled": "failed",
                "timeout": "DETECT_OBJECTS",
            },
        )
        # self.add_state(
        #     'SLEEP',
        #     Wait(wait_time=4),
        #     transitions={'succeeded': 'DETECT_OBJECTS', 'failed': 'failed'}
        # )
        self.add_state(
            "DETECT_OBJECTS",
            Detect3DInArea(
                area_polygon=polygon,
                filter=object_filter,
                model=model,
                z_min=-10,
                z_max=10.0,
                confidence=min_confidence,
                target_frame="map",
            ),
            transitions={"succeeded": "PROCESS_DETECTIONS", "failed": "failed"},
        )
        self.add_state(
            "PROCESS_DETECTIONS",
            ProcessDetections(min_new_object_dist=min_new_object_dist),
            transitions={"succeeded": "GET_LOOK_POINT", "failed": "failed"},
        )

    def _get_look_point(self, blackboard) -> str:
        """
        Callback to get the look point based on the current sweep point index.

        Args:
            userdata (smach.UserData): User data containing the sweep points and index.

        Returns:
            str: Outcome of the state, "succeeded".
        """
        index = blackboard["sweep_point_index"]
        yasmin.YASMIN_LOG_INFO(index)
        if index < len(blackboard["sweep_points"]):
            blackboard["pointstamped"] = blackboard["sweep_points"][index]
            blackboard["sweep_point_index"] += 1
            return "continue"
        else:
            yasmin.YASMIN_LOG_INFO("Finished iterating through sweep points.")
            return "succeeded"


class DetectAllInPolygon(yasmin.StateMachine):
    """
    State machine to sweep and detect all objects within
    a given polygon. For now, the Z-axis is ignored, and we assume
    that the sweet is performed at a fixed height, across fixed points.
    """

    _polygon: ShapelyPolygon
    _min_coverage: float
    _object_filter: Optional[List[str]]
    _model: str
    _models: Optional[List[str]]
    _min_confidence: float
    _min_new_object_dist: float
    _debug_publisher: Publisher
    _prompt: Optional[str]

    def __init__(
        self,
        polygon: ShapelyPolygon,
        min_coverage: float = 0.8,
        model: str = "yolo11n-seg.pt",
        object_filter: Optional[List[str]] = None,
        min_confidence: float = 0.5,
        min_new_object_dist: float = 0.1,
        use_lang_sam: bool = False,
        prompt: Optional[str] = None,
    ):
        """
        Args:

            polygon (ShapelyPolygon): Polygon to sweep and detect objects in.

            min_coverage (float, optional): Mininum coverage of the polygon from the sweep.
            Defaults to 0.8.

            object_filter (Optional[List[str]], optional): Optional list of object names to detect.
            Defaults to None, meaning all objects will be detected.

            min_confidence (float, optional): Minimum YOLO confidence for detecting an object.
            Defaults to 0.5.

            min_new_object_dist (float, optional): Minimum distance between detected
            objects of the same class in order to count a detection as a new object.
            Defaults to 0.1.

            use_lang_sam (bool, optional): Whether to use the LangSam detection model,
            if true, requires a prompt. Defaults to False, mneaning use YOLO instead.

            prompt (Optional[str], optional): Prompt for the LangSam model, if used.
        """

        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)

        self.add_output_key("detected_objects")

        self._polygon = polygon
        self._min_coverage = min_coverage
        self._object_filter = object_filter
        self._model = model
        self._min_confidence = min_confidence
        self._min_new_object_dist = min_new_object_dist
        image_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._debug_publisher = self._node.create_publisher(
            Image, "/detect_all_in_polygon/debug", image_qos
        )
        self._prompt = prompt
        if use_lang_sam:
            assert (
                self._prompt is not None
            ), "Prompt must be provided for LangSam model."

        self.build_state_machine()

    def _publish_detected_objects(self, blackboard) -> Image:
        """Create the debug image that PublisherState will publish."""
        images_for_tiling = []
        for image_raw, detections in blackboard["debug_images"]:
            yasmin_ros.logger_node.get_logger().info(
                f"Processing {len(detections)} detections for debug image."
            )
            if not detections:
                continue

            cv2_image = msg_to_cv2_img(image_raw)
            for detection in detections:
                xywh = detection.xywh
                label = detection.name
                confidence = detection.confidence
                cv2.rectangle(
                    cv2_image,
                    (int(xywh[0]), int(xywh[1])),
                    (int(xywh[0] + xywh[2]), int(xywh[1] + xywh[3])),
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    cv2_image,
                    f"{label} {confidence:.2f}",
                    (int(xywh[0]), int(xywh[1] - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )
            images_for_tiling.append(cv2_image)

        if not images_for_tiling:
            yasmin_ros.logger_node.get_logger().warn(
                "No debug images with detections to publish."
            )
            return Image()

        tiled_image = cv2.hconcat(images_for_tiling)
        yasmin_ros.logger_node.get_logger().info("Created debug image with detections.")
        return cv2_img_to_msg(tiled_image)

    def build_state_machine(self):
        """
        Builds the state machine for detecting all objects in the polygon.
        """

        # State to calculate the points to sweep

        publish_detected_objects = yasmin_ros.PublisherState(
            msg_type=Image,
            topic_name="/detect_all_in_polygon/debug",
            create_message_handler=self._publish_detected_objects,
        )
        for input in ["debug_images", "detected_objects"]:
            publish_detected_objects.add_input_key(input)

        self.add_state(
            "CALCULATE_SWEEP_POINTS",
            CalculateSweepPoints(
                polygon=self._polygon,
                min_coverage=self._min_coverage,
            ),
            transitions={"succeeded": "LOOK_AND_DETECT", "failed": "failed"},
        )
        self.add_state(
            "LOOK_AND_DETECT",
            IterateThroughPoints(
                polygon=self._polygon,
                model=self._model,
                object_filter=self._object_filter,
                min_confidence=self._min_confidence,
                min_new_object_dist=self._min_new_object_dist,
            ),
            transitions={"succeeded": "PUBLISH_DETECTED_OBJECTS", "failed": "failed"},
        )

        self.add_state(
            "PUBLISH_DETECTED_OBJECTS",
            publish_detected_objects,
            transitions={"succeeded": "succeeded"},
        )


def main():
    seat_area = [
        [0.9422937035560608, -1.9376981258392334],
        [-0.01625092327594757, -1.1312360763549805],
        [-0.5108118057250977, -1.6913851499557495],
        [0.4300234913825989, -2.5222253799438477],
    ]

    seat_polygon = ShapelyPolygon(seat_area)

    rclpy.init()

    # node = Node('Detect_All_In_Polygon')
    # executor = Executor()
    # executor.add_node(node)

    # thread = Thread(target=executor.spin())
    # thread.start()
    yasmin_ros.set_ros_loggers()

    bb = Blackboard()
    bb["sweep_points"] = []
    bb["detected_objects"] = []
    bb["debug_images"] = []
    bb["pointstamped"] = PointStamped()
    bb["sweep_point_index"] = 0

    sm = yasmin.StateMachine(outcomes=["succeeded", "failed"], handle_sigint=True)
    sm.add_state(
        "DETECT_ALL_IN_POLYGON",
        DetectAllInPolygon(
            polygon=seat_polygon,
            min_coverage=1.0,
            min_new_object_dist=0.40,
            min_confidence=0.7,
            object_filter=["person", "chair"],
        ),
        transitions={"succeeded": "succeeded", "failed": "failed"},
    )

    YasminViewerPub(sm, "YASMIN_MULTIPLE_STATES_DEMO")
    try:
        outcome = sm(bb)

        yasmin.YASMIN_LOG_INFO(f"SM finished with outcome: {outcome}")
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
