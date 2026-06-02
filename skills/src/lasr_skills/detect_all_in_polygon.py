#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message

import yasmin
import yasmin_ros
from yasmin import Blackboard

# import tf2_ros as tf
import numpy as np
import cv2
from threading import Thread

from time import sleep

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

from .wait import Wait

class ProcessDetections(yasmin.State):
    """
    State to process the detected objects and filter them based on the
    minimum distance between objects of the same class.
    """

    _min_new_object_dist: float

    def __init__(self, min_new_object_dist: float = 0.1):
        super().__init__(
            outcomes=["succeeded", "failed"]
        )
        
        self.add_input_key('detections_3d')
        self.add_input_key('detected_objects')
        self.add_input_key('image_raw')
        self.add_input_key('debug_images')
        
        self.add_output_key('detected_objects')
        self.add_output_key('debug_images')
        
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
            for detection in blackboard['detections_3d']:
                if detection in blackboard['detected_objects']:
                    continue

                # Check if the detection is a new object
                is_new_object = True
                for existing_detection in blackboard['detected_objects']:
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

            blackboard['debug_images'].append((blackboard['image_raw'], new_detections))
            blackboard['detected_objects'].extend(new_detections)
            yasmin_ros.logger_node.get_logger().info(
                f"Processed detections. Total detected objects: {len(blackboard['detected_objects'])}"
            )
            yasmin_ros.logger_node.get_logger().info("Detected objects:")
            for obj in blackboard['detected_objects']:
                yasmin_ros.logger_node.get_logger().info(
                    f" - {obj.name} at ({obj.point.x}, {obj.point.y}, {obj.point.z})"
                )
            return "succeeded"
        except Exception as e:
            yasmin_ros.logger_node.get_logger().error(f"Failed to process detections: {e}")
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
        super().__init__(
            outcomes=["succeeded", "failed"]
        )
        
        self.add_input_key('sweep_points')
        self.add_input_key('detected_objects')
        
        self.add_output_key('sweep_points')
        
        self._polygon = polygon
        self._min_coverage = min_coverage
        self._z_axis = z_axis
        self._fov_depth = fov_depth

        self._tf_buffer = tf2_ros.Buffer(Duration(seconds=10.0))
        node = yasmin_ros.logger_node
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, yasmin_ros.logger_node)

    def _get_camera_fov_polygon(self) -> ShapelyPolygon:
        """
        Projects the camera's FOV to the ground plane using intrinsics and TF.

        Returns:
            ShapelyPolygon: Footprint of camera FOV in map frame.
        """

        success, camera_info = wait_for_message(
            CameraInfo, yasmin_ros.logger_node, "/head_front_camera/depth/camera_info"
        )
        
        
        while not success or camera_info is None:
            yasmin.YASMIN_LOG_INFO("Waiting for camera info")
            sleep(1)


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
                    "odom",
                    camera_info.header.frame_id,
                    Time(),
                    timeout=Duration(seconds=5.0),
                )
                point_map = do_transform_point(point_cam, transform)
                transformed_points.append((point_map.point.x, point_map.point.y))
            except Exception as e:
                yasmin_ros.logger_node.get_logger().error(
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

            yasmin_ros.logger_node.get_logger().info(
                f"Selected new footprint, total coverage: {covered.area / total_area:.2%}, score: {best_score:.2f}"
            )

        return selected, covered

    def _calculate_sweep_points(self) -> List[PointStamped]:
        """
        Calculates the points to sweep based on the polygon and FOV projection.

        Returns:
            List[PointStamped]: List of sweep points in map frame.
        """
        yasmin_ros.logger_node.get_logger().info("Waiting for camera info and TF to map frame...")
        fov_polygon = self._get_camera_fov_polygon()

        # Optional: visualize FOV

        qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )  # Verify publisher durability profile (https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)

        pub = yasmin_ros.logger_node.create_publisher(PolygonStamped, "projected_fov_polygon", qos)

        pub.publish(
            PolygonStamped(
                header=Header(
                    frame_id="odom", stamp=yasmin_ros.logger_node.get_clock().now().to_msg()
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
                header=Header(frame_id="odom"),
                point=Point(x=fp.centroid.x, y=fp.centroid.y, z=self._z_axis),
            )
            for fp in selected_footprints
        ]
        yasmin_ros.logger_node.get_logger().info(f"Calculated {len(sweep_points)} sweep points.")
        return sweep_points

    def execute(self, blackboard) -> str:
        """Main SMACH execution entrypoint"""
        blackboard['sweep_points'] = self._calculate_sweep_points()
        return "succeeded"

class IterateThroughPoints(yasmin.StateMachine):
    def __init__(self, 
                polygon: ShapelyPolygon,
                model: str = "yolo11n-seg.pt",
                models: Optional[List[str]] = None,
                object_filter: Optional[List[str]] = None,
                min_confidence: float = 0.5):
        super().__init__(outcomes=['succeeded', 'failed'], handle_sigint=True)
        
        input_keys=["sweep_points", "sweep_point_index", "detected_objects", "pointstamped"]
        
        get_point_state = yasmin.CbState(outcomes=['succeeded', 'failed', 'continue'], callback=self._get_look_point)
        for key in input_keys:
            get_point_state.add_input_key(key)
        get_point_state.add_output_key('pointstamped')
        
        
        self.add_state(
            'GET_LOOK_POINT',
            get_point_state,
            transitions={'succeeded': 'succeeded', 'continue': 'LOOK_POINT', 'failed': 'failed'}
        )
        self.add_state(
            'LOOK_POINT',
            LookToPoint(),
            transitions={'succeeded': 'SLEEP', 'aborted': 'failed', 'canceled': 'failed', 'timeout': 'failed'}
        )
        self.add_state(
            'SLEEP',
            Wait(wait_time=4),
            transitions={'succeeded': 'DETECT_OBJECTS', 'failed': 'failed'}
        )
        self.add_state(
            'DETECT_OBJECTS',
            Detect3DInArea(
                            area_polygon=polygon,
                            filter=object_filter,
                            model=model,
                            models=models,
                            z_min=0.0,
                            z_max=10.0,
                            confidence=min_confidence,
                            target_frame='odom'
                        ),
            transitions={'succeeded': 'PROCESS_DETECTIONS', 'failed': 'failed'}
        )
        self.add_state(
            'PROCESS_DETECTIONS',
            ProcessDetections(),
            transitions={'succeeded': 'GET_LOOK_POINT', 'failed': 'failed'}
        )
        
    def _get_look_point(self, blackboard) -> str:
        """
        Callback to get the look point based on the current sweep point index.

        Args:
            userdata (smach.UserData): User data containing the sweep points and index.

        Returns:
            str: Outcome of the state, "succeeded".
        """
        index = blackboard['sweep_point_index']
        yasmin.YASMIN_LOG_INFO(index)
        if index < len(blackboard['sweep_points']):
            blackboard['pointstamped'] = blackboard['sweep_points'][index]
            yasmin_ros.logger_node.get_logger().info(
                f"Look point set to: {blackboard['pointstamped']}"
            )
            blackboard['sweep_point_index'] += 1
            return "continue"
        else:
            yasmin_ros.logger_node.get_logger().error("Index out of bounds for sweep points.")
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
        models: Optional[List[str]] = None,
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

        super().__init__(
            outcomes=["succeeded", "failed"], handle_sigint=True
        )
        
        self.add_output_key('detected_objects')
        
        self._polygon = polygon
        self._min_coverage = min_coverage
        self._object_filter = object_filter
        self._model = model
        self._models = models
        self._min_confidence = min_confidence
        self._min_new_object_dist = min_new_object_dist
        self._prompt = prompt
        if use_lang_sam:
            assert (
                self._prompt is not None
            ), "Prompt must be provided for LangSam model."

        self.build_state_machine()

    def _publish_detected_objects(self, blackboard) -> str:
        """
        Callback to publish the detected objects.

        Args:
            userdata (smach.UserData): User data containing the detected objects.

        Returns:
            str: Outcome of the state, "succeeded".
        """
        images_for_tiling = []
        for image_raw, detections in blackboard['debug_images']:
            print(f"Processing {len(detections)} detections for image.")
            if not detections:
                yasmin_ros.logger_node.get_logger().warn("No detections to publish.")
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
            yasmin_ros.logger_node.get_logger().info("Published debug images with detections.")

        return "succeeded"

    def build_state_machine(self):
        """
        Builds the state machine for detecting all objects in the polygon.
        """

        # State to calculate the points to sweep
        
        
        publish_detected_objects = yasmin_ros.PublisherState(msg_type=Image, topic_name='/detect_all_in_polygon/debug', create_message_handler=self._publish_detected_objects)
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
        self.add_state('LOOK_AND_DETECT',
                       IterateThroughPoints(polygon=self._polygon, object_filter=self._object_filter),
                       transitions={'succeeded': 'PUBLISH_DETECTED_OBJECTS', 'failed': 'failed'})

        self.add_state(
            "PUBLISH_DETECTED_OBJECTS",
            publish_detected_objects,
            transitions={"succeeded": "succeeded"}
        )


def main():
    seat_area = [
        [2.21, -2.15],
        [2.45, -2.45],
        [1.23, -2.42],
        [1.31, -2.06],
    ]

    seat_polygon = ShapelyPolygon(seat_area)

    rclpy.init()
    
    yasmin_ros.set_ros_loggers()
    
    bb = Blackboard()
    bb['sweep_points'] = []
    bb['detected_objects'] = []
    bb['debug_images'] = []
    bb['look_point'] = PointStamped()
    bb['sweep_point_index'] = 0
    
    sm = yasmin.StateMachine(outcomes=['succeeded', 'failed'], handle_sigint=True)
    sm.add_state(
        'DETECT_ALL_IN_POLYGON',
        DetectAllInPolygon(polygon=seat_polygon, object_filter=['person', 'chair']),
        transitions={'succeeded': 'succeeded', 'failed': 'failed'}
    )
    
    outcome = sm(bb)
    yasmin.YASMIN_LOG_INFO(f'SM finished with outcome: {outcome}')
        
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
