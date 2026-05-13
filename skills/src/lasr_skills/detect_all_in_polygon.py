#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message

import smach
from smach_ros import RosState

# import tf2_ros as tf
import numpy as np
import cv2
from threading import Thread

# from tf_pcl import pcl_transform
from typing import List, Optional, Tuple

# from shapely import MultiPoint
from shapely import Polygon as ShapelyPolygon
from shapely import Point as ShapelyPoint
from shapely.affinity import translate
from sensor_msgs.msg import Image  # , PointCloud2 as pc2
from cv2_img import msg_to_cv2_img, cv2_img_to_msg
from geometry_msgs.msg import Point, PointStamped
from lasr_vision_interfaces.msg import Detection3D

from .look_to_point import LookToPoint
from .detect_3d_in_area import Detect3DInArea


class ProcessDetections(RosState):
    """
    State to process the detected objects and filter them based on the
    minimum distance between objects of the same class.
    """

    _min_new_object_dist: float

    def __init__(self, node: Node, min_new_object_dist: float = 0.1):
        RosState.__init__(
            self,
            node=node,
            outcomes=["succeeded", "failed"],
            input_keys=[
                "detections_3d",
                "detected_objects",
                "image_raw",
                "debug_images",
            ],
            output_keys=["detected_objects", "debug_images"],
        )
        self._min_new_object_dist = min_new_object_dist

    def execute(self, userdata: smach.UserData) -> str:
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
            for detection in userdata.detections_3d:
                if detection in userdata.detected_objects:
                    continue

                # Check if the detection is a new object
                is_new_object = True
                for existing_detection in userdata.detected_objects:
                    if (
                        existing_detection.name == detection.name
                        and euclidean_distance(
                            existing_detection.point, detection.point
                        )
                        < self._min_new_object_dist
                    ):
                        self.node.get_logger().info(
                            f"Detected object {detection.name} is too close to existing object {existing_detection.name}. Not counting as new."
                        )
                        is_new_object = False
                        break

                if is_new_object:
                    new_detections.append(detection)

            userdata.debug_images.append((userdata.image_raw, new_detections))
            userdata.detected_objects.extend(new_detections)
            self.node.get_logger().info(
                f"Processed detections. Total detected objects: {len(userdata.detected_objects)}"
            )
            self.node.get_logger().info("Detected objects:")
            for obj in userdata.detected_objects:
                self.node.get_logger().info(
                    f" - {obj.name} at ({obj.point.x}, {obj.point.y}, {obj.point.z})"
                )
            return "succeeded"
        except Exception as e:
            self.node.get_logger().error(f"Failed to process detections: {e}")
            return "failed"


import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.publisher import Publisher
from rclpy.executors import MultiThreadedExecutor

import smach
from smach_ros import RosState

import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point

# import tf2_geometry_msgs
import numpy as np

from std_msgs.msg import Header
from geometry_msgs.msg import (
    Point,
    Point32,
    PointStamped,
    Polygon as ROSPolygon,
    PolygonStamped,
)
from shapely.geometry import (
    Polygon as ShapelyPolygon,
    Point as ShapelyPoint,
    # MultiPoint,
)
from shapely.affinity import translate
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
from typing import List, Tuple
import threading


class CalculateSweepPoints(RosState):
    """
    State to calculate the points to sweep based on the polygon and camera FOV.
    """

    def __init__(
        self,
        node: Node,
        polygon: ShapelyPolygon,
        min_coverage: float = 0.8,
        z_axis: float = 0.7,
        fov_depth: float = 2.0,
    ):
        RosState.__init__(
            self,
            node=node,
            outcomes=["succeeded", "failed"],
            output_keys=["sweep_points"],
            input_keys=["sweep_points", "detected_objects"],
        )
        self._polygon = polygon
        self._min_coverage = min_coverage
        self._z_axis = z_axis
        self._fov_depth = fov_depth

        self._tf_buffer = tf2_ros.Buffer(Duration(seconds=10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self.node)

    def _get_camera_fov_polygon(self) -> ShapelyPolygon:
        """
        Projects the camera's FOV to the ground plane using intrinsics and TF.

        Returns:
            ShapelyPolygon: Footprint of camera FOV in map frame.
        """

        success, camera_info = wait_for_message(
            CameraInfo, self.node, "/head_front_camera/depth/camera_info"
        )
        if not success or camera_info is None:
            self.node.get_logger().warn("Timed out waiting for camera info")

        model = PinholeCameraModel()
        model.fromCameraInfo(camera_info)

        # Define pixel corners (image boundaries)
        corners = [
            (0, 0),  # top-left
            (model.width, 0),  # top-right
            (model.width, model.height),  # bottom-right
            (0, model.height),  # bottom-left
        ]

        # Transform pixel rays to map frame
        transformed_points = []
        for u, v in corners:
            ray = model.projectPixelTo3dRay((u, v))
            point_cam = PointStamped()
            point_cam.header.frame_id = camera_info.header.frame_id
            point_cam.header.stamp = Time().to_msg()
            point_cam.point.x = ray[0] * self._fov_depth
            point_cam.point.y = ray[1] * self._fov_depth
            point_cam.point.z = ray[2] * self._fov_depth

            # Transform to map frame
            try:
                transform = self._tf_buffer.lookup_transform(
                    "map",
                    camera_info.header.frame_id,
                    Time(),
                    timeout=Duration(seconds=1.0),
                )
                point_map = do_transform_point(point_cam, transform)
                transformed_points.append((point_map.point.x, point_map.point.y))
            except Exception as e:
                self.node.get_logger().error(
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

            self.node.get_logger().info(
                f"Selected new footprint, total coverage: {covered.area / total_area:.2%}, score: {best_score:.2f}"
            )

        return selected, covered

    def _calculate_sweep_points(self) -> List[PointStamped]:
        """
        Calculates the points to sweep based on the polygon and FOV projection.

        Returns:
            List[PointStamped]: List of sweep points in map frame.
        """
        self.node.get_logger().info("Waiting for camera info and TF to map frame...")
        fov_polygon = self._get_camera_fov_polygon()

        # Optional: visualize FOV

        qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )  # Verify publisher durability profile (https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)

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
        self.node.get_logger().info(f"Calculated {len(sweep_points)} sweep points.")
        return sweep_points

    def execute(self, userdata: smach.UserData) -> str:
        """Main SMACH execution entrypoint"""
        try:
            userdata.sweep_points = self._calculate_sweep_points()
            return "succeeded"
        except Exception as e:
            self.node.get_logger().error(f"Failed to calculate sweep points: {e}")
            return "failed"


class DetectAllInPolygon(smach.StateMachine):
    """
    State machine to sweep and detect all objects within
    a given polygon. For now, the Z-axis is ignored, and we assume
    that the sweet is performed at a fixed height, across fixed points.
    """

    _node: Node
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
        node: Node,
        polygon: ShapelyPolygon,
        min_coverage: float = 0.8,
        model: str = "yolo11n-seg.pt",
        models: Optional[List[str]] = None,
        object_filter: Optional[List[str]] = None,
        min_confidence: float = 0.5,
        min_new_object_dist: float = 0.1,
        use_lang_sam: bool = False,
        prompt: Optional[str] = None,
    ):
        """
        Args:
            node: A rclpy node.

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

        super().__init__(
            outcomes=["succeeded", "failed"], output_keys=["detected_objects"]
        )

        self._node = node
        self._polygon = polygon
        self._min_coverage = min_coverage
        self._object_filter = object_filter
        self._model = model
        self._models = models
        self._min_confidence = min_confidence
        self._min_new_object_dist = min_new_object_dist
        self._debug_publisher = self._node.create_publisher(
            Image, "/detect_all_in_polygon/debug", 10
        )
        self._prompt = prompt
        if use_lang_sam:
            assert (
                self._prompt is not None
            ), "Prompt must be provided for LangSam model."

        with self:
            self.build_state_machine()

    def _get_look_point(self, userdata: smach.UserData) -> str:
        """
        Callback to get the look point based on the current sweep point index.

        Args:
            userdata (smach.UserData): User data containing the sweep points and index.

        Returns:
            str: Outcome of the state, "succeeded".
        """
        index = userdata.sweep_point_index
        if index < len(userdata.sweep_points):
            userdata.look_point = userdata.sweep_points[index]
            self._node.get_logger().info(
                f"Look point set to: {userdata.look_point.point.x}, {userdata.look_point.point.y}, {userdata.look_point.point.z}"
            )
            return "succeeded"
        else:
            self._node.get_logger().error("Index out of bounds for sweep points.")
            return "failed"

    def _nap(self, userdata) -> str:
        """
        Callback to sleep for a given duration.

        Args:
            duration (float, optional): Duration to sleep in seconds. Defaults to 1.0.

        Returns:
            str: Outcome of the state, "succeeded".
        """
        rclpy.spin_once(self._node, timeout_sec=0.25)
        return "succeeded"

    def _publish_detected_objects(self, userdata: smach.UserData) -> str:
        """
        Callback to publish the detected objects.

        Args:
            userdata (smach.UserData): User data containing the detected objects.

        Returns:
            str: Outcome of the state, "succeeded".
        """
        images_for_tiling = []
        for image_raw, detections in userdata.debug_images:
            print(f"Processing {len(detections)} detections for image.")
            if not detections:
                self._node.get_logger().warn("No detections to publish.")
                continue
            cv2_image = msg_to_cv2_img(image_raw)
            # Loop over each detection, annotate image with bounding boxes
            # tile images, and publish
            for detection in detections:
                xywh = detection.xywh
                label = detection.name
                confidence = detection.confidence
                # Annotate the image with bounding box and label
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
        if images_for_tiling:
            # Create a tiled image from the list of images
            tiled_image = cv2.hconcat(images_for_tiling)
            # Convert to ROS Image message
            image_msg = cv2_img_to_msg(tiled_image)
            # Publish the tiled image
            self._debug_publisher.publish(image_msg)
            self._node.get_logger().info("Published debug images with detections.")

        return "succeeded"

    def build_state_machine(self):
        """
        Builds the state machine for detecting all objects in the polygon.
        """

        # State to calculate the points to sweep
        self.userdata.sweep_points = []
        self.userdata.detected_objects = []
        self.userdata.debug_images = []
        self.userdata.look_point = PointStamped()
        self.add(
            "CALCULATE_SWEEP_POINTS",
            CalculateSweepPoints(
                node=self._node,
                polygon=self._polygon,
                min_coverage=self._min_coverage,
            ),
            transitions={"succeeded": "LOOK_AND_DETECT", "failed": "failed"},
            remapping={"sweep_points": "sweep_points"},
        )

        look_and_detect_iterator = smach.Iterator(
            outcomes=["succeeded", "failed"],
            input_keys=[
                "sweep_points",
                "detected_objects",
                "look_point",
                "debug_images",
            ],
            output_keys=["detected_objects", "debug_images"],
            it=lambda: range(0, len(self.userdata.sweep_points)),
            it_label="sweep_point_index",
            exhausted_outcome="succeeded",
        )

        with look_and_detect_iterator:
            container_sm = smach.StateMachine(
                outcomes=["continue", "failed", "succeeded"],
                input_keys=[
                    "sweep_points",
                    "sweep_point_index",
                    "detected_objects",
                    "look_point",
                    "debug_images",
                ],
                output_keys=[
                    "look_point",
                    "detections_3d",
                    "image_raw",
                    "detected_objects",
                    "debug_images",
                ],
            )
            with container_sm:
                smach.StateMachine.add(
                    "GET_LOOK_POINT",
                    smach.CBState(
                        self._get_look_point,
                        output_keys=["look_point"],
                        outcomes=["succeeded", "failed"],
                        input_keys=[
                            "sweep_points",
                            "sweep_point_index",
                            "detected_objects",
                            "look_point",
                        ],
                    ),
                    transitions={"succeeded": "LOOK_POINT", "failed": "failed"},
                    remapping={"look_point": "pointstamped"},
                )
                smach.StateMachine.add(
                    "LOOK_POINT",
                    LookToPoint(self._node),
                    transitions={
                        "succeeded": "SLEEP",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )
                smach.StateMachine.add(
                    "SLEEP",
                    smach.CBState(
                        self._nap,
                        outcomes=["succeeded"],
                        input_keys=["look_point"],
                    ),
                    transitions={"succeeded": "DETECT_OBJECTS"},
                )
                if self._prompt is not None:
                    pass
                    # smach.StateMachine.add(
                    #     "DETECT_OBJECTS",
                    #     Detect3DInAreaLangSam(
                    #         area_polygon=self._polygon,
                    #         box_threshold=self._min_confidence,
                    #         text_threshold=self._min_confidence,
                    #         target_frame="map",
                    #         prompt=self._prompt,
                    #     ),
                    #     transitions={
                    #         "succeeded": "PROCESS_DETECTIONS",
                    #         "failed": "failed",
                    #     },
                    #     remapping={
                    #         "lang_sam_detections_3d": "detections_3d",
                    #         "image_raw": "image_raw",
                    #     },
                    # )
                else:
                    smach.StateMachine.add(
                        "DETECT_OBJECTS",
                        Detect3DInArea(
                            node=self._node,
                            area_polygon=self._polygon,
                            filter=self._object_filter,
                            model=self._model,
                            models=self._models,
                            z_min=0.0,
                            z_max=10.0,
                            confidence=self._min_confidence,
                        ),
                        transitions={
                            "succeeded": "PROCESS_DETECTIONS",
                            "failed": "failed",
                        },
                        remapping={
                            "detections_3d": "detections_3d",
                            "image_raw": "image_raw",
                        },
                    )
                smach.StateMachine.add(
                    "PROCESS_DETECTIONS",
                    ProcessDetections(
                        node=self._node, min_new_object_dist=self._min_new_object_dist
                    ),
                    transitions={"succeeded": "continue", "failed": "failed"},
                    remapping={
                        "detections_3d": "detections_3d",
                        "detected_objects": "detected_objects",
                    },
                )
            smach.Iterator.set_contained_state(
                "CONTAINER_SM",
                container_sm,
                loop_outcomes=["continue"],
            )
        self.add(
            "LOOK_AND_DETECT",
            look_and_detect_iterator,
            transitions={
                "succeeded": "PUBLISH_DETECTED_OBJECTS",
                "failed": "failed",
            },
            remapping={
                "sweep_points": "sweep_points",
                "detected_objects": "detected_objects",
            },
        )

        self.add(
            "PUBLISH_DETECTED_OBJECTS",
            smach.CBState(
                self._publish_detected_objects,
                input_keys=["debug_images", "detected_objects"],
                outcomes=["succeeded"],
            ),
            transitions={"succeeded": "succeeded"},
            remapping={
                "debug_images": "debug_images",
                "detected_objects": "detected_objects",
            },
        )


def main():
    seat_area = [
        [2.260225772857666, 1.446178674697876],
        [2.342136859893799, -0.8490455150604248],
        [0.5737614631652832, -0.8957526683807373],
        [0.7276101112365723, 1.311692714691162],
    ]

    seat_polygon = ShapelyPolygon(seat_area)

    rclpy.init()
    node = rclpy.create_node("detect_all_in_polygon")

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    sm = DetectAllInPolygon(
        node,
        seat_polygon,
        object_filter=["person", "chair"],
        min_coverage=1.0,
        min_new_object_dist=0.40,
        min_confidence=0.7,
    )
    outcome = sm.execute()
    node.get_logger().info(f"State machine finished with outcome: {outcome}")


if __name__ == "__main__":
    main()
