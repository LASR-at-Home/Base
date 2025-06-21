import rospy
import smach
import tf2_ros as tf
import numpy as np
import sensor_msgs.point_cloud2 as pc2

from tf_pcl import pcl_transform
from typing import List, Optional, Tuple
from shapely import MultiPoint
from shapely import Polygon as ShapelyPolygon
from shapely import Point as ShapelyPoint
from shapely.affinity import translate
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PointStamped

from lasr_skills import LookToPoint, Detect3DInArea


class ProcessDetections(smach.State):
    """
    State to process the detected objects and filter them based on the
    minimum distance between objects of the same class.
    """

    _min_new_object_dist: float

    def __init__(self, min_new_object_dist: float = 0.1):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["detections_3d", "detected_objects"],
            output_keys=["detected_objects"],
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

        try:
            for detection in userdata.detections_3d:
                if detection in userdata.detected_objects:
                    continue

                # Check if the detection is a new object
                is_new_object = True
                for existing_detection in userdata.detected_objects:
                    if (
                        existing_detection.class_name == detection.class_name
                        and euclidean_distance(
                            existing_detection.point, detection.point
                        )
                        < self._min_new_object_dist
                    ):
                        is_new_object = False
                        break

                if is_new_object:
                    userdata.detected_objects.append(detection)

            return "succeeded"
        except Exception as e:
            rospy.logerr(f"Failed to process detections: {e}")
            return "failed"


class CalculateSweepPoints(smach.State):
    """
    State to calculate the points to sweep based on the polygon.
    """

    _polygon: ShapelyPolygon
    _min_coverage: float
    _tf_buffer: tf.Buffer
    _z_axis: float  # Fixed height for the sweep

    def __init__(
        self,
        polygon: ShapelyPolygon,
        min_coverage: float = 0.8,
        z_axis: float = 0.5,
    ):

        smach.State.__init__(
            self, outcomes=["succeeded", "failed"], output_keys=["sweep_points"]
        )
        self._polygon = polygon
        self._min_coverage = min_coverage
        self._z_axis = z_axis

        self._tf_buffer = tf.Buffer(cache_time=rospy.Duration(10))
        tf.TransformListener(self._tf_buffer)

    def _sample_points_in_polygon(
        self, polygon: ShapelyPolygon, num_samples: float = 1000
    ) -> List[ShapelyPoint]:
        """Randomly samples points within a polygon.

        Args:
            polygon (ShapelyPolygon): _description_
            num_samples (float, optional): _description_. Defaults to 1000.

        Returns:
            _type_: _description_
        """
        minx, miny, maxx, maxy = polygon.bounds
        samples: List[ShapelyPoint] = []
        while len(samples) < num_samples:
            p = Point(np.random.uniform(minx, maxx), np.random.uniform(miny, maxy))
            if polygon.contains(p):
                samples.append(p)
        return samples

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
        return translate(rel_hull, xoff=look_point[0], yoff=look_point[1])

    def _greedy_coverage_min_overlap(
        self,
        candidate_footprints,
        coverage_goal=0.9,
        overlap_penalty=0.5,
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

            rospy.loginfo(
                f"Selected new footprint, total coverage: {covered.area / total_area:.2%}, score: {best_score:.2f}"
            )

        return selected, covered

    def _calculate_sweep_points(self) -> List[Point]:
        """
        Calculates the points to sweep based on the polygon and minimum coverage.

        Returns:
            List[Point]: List of points to sweep.
        """
        # Get area of the camera view, in the map frame
        pcl = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        trans = self._tf_buffer.lookup_transform(
            "map",
            pcl.header.frame_id,
            rospy.Time(0),
            rospy.Duration(1.0),
        )
        pcl_map = pcl_transform(pcl, trans)

        # Project into 2D to get area of camera view in map frame
        points = np.array(
            [
                [p[0], p[1]]
                for p in pc2.read_points(
                    pcl_map, field_names=["x", "y"], skip_nans=True
                )
            ]
        )
        camera_view = MultiPoint(points)
        camera_hull = camera_view.convex_hull
        relative_camera_hull = self._extract_relative_footprint(camera_hull)
        sampled_points = self._sample_points_in_polygon(self._polygon, num_samples=1000)
        candidate_footprints = [
            self._place_footprint_at_point(relative_camera_hull, p)
            for p in sampled_points
        ]
        look_at_points, _ = self._greedy_coverage_min_overlap(
            candidate_footprints,
            coverage_goal=self._min_coverage,
        )

        sweep_points = [
            PointStamped(
                header=rospy.Header(frame_id="map"),
                point=Point(fp.centroid.x, fp.centroid.y, self._z_axis),
            )
            for fp in look_at_points
        ]

        return sweep_points

    def execute(self, userdata: smach.UserData) -> str:
        """Calculates the points to look to, in order to sweep the polygon.


        Args:
            userdata (smach.UserData): User data to store the sweep points.

        Returns:
            str: Outcome of the state, "succeeded" or "failed".
        """

        # Calculate the points to sweep based on the polygon
        try:
            userdata.sweep_points = self._calculate_sweep_points()
            return "succeeded"
        except Exception as e:
            rospy.logerr(f"Failed to calculate sweep points: {e}")
            return "failed"


class DetectAllInPolygon(smach.StateMachine):
    """
    State machine to sweep and detect all objects within
    a given polygon. For now, the Z-axis is ignored, and we assume
    that the sweet is performed at a fixed height, across fixed points.
    """

    _polygon: ShapelyPolygon
    _min_coverage: float
    _object_filter: Optional[List[str]]
    _min_confidence: float
    _min_new_object_dist: float

    def __init__(
        self,
        polygon: ShapelyPolygon,
        min_coverage: float = 0.8,
        object_filter: Optional[List[str]] = None,
        min_confidence: float = 0.5,
        min_new_object_dist: float = 0.1,
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
        """

        super().__init__(outcomes=["succeeded", "failed"])

        self._polygon = polygon
        self._min_coverage = min_coverage
        self._object_filter = object_filter
        self._min_confidence = min_confidence
        self._min_new_object_dist = min_new_object_dist

        with self:
            self.build_state_machine()

    def build_state_machine(self):
        """
        Builds the state machine for detecting all objects in the polygon.
        """

        # State to calculate the points to sweep
        self.add(
            "CALCULATE_SWEEP_POINTS",
            CalculateSweepPoints(
                polygon=self._polygon,
                min_coverage=self._min_coverage,
            ),
            transitions={"succeeded": "LOOK_AND_DETECT", "failed": "failed"},
            remapping={"sweep_points": "sweep_points"},
        )

        look_and_detect_iterator = smach.Iterator(
            outcomes=["succeeded", "failed"],
            input_keys=["sweep_points"],
            output_keys=["detected_objects"],
            it=lambda: range(0, len(self.userdata.sweep_points)),
            it_label="sweep_point_index",
            exhausted_outcome="succeeded",
        )

        with look_and_detect_iterator:
            container_sm = smach.StateMachine(
                outcomes=["continue", "failed", "succeeded"],
                input_keys=["sweep_points", "sweep_point_index", "detected_objects"],
                output_keys=[
                    "look_point",
                    "detections_3d",
                    "image_raw",
                    "detected_objects",
                ],
            )
            with container_sm:
                smach.StateMachine.add(
                    "GET_LOOK_POINT",
                    smach.CBState(
                        lambda userdata: userdata.sweep_points[
                            userdata.sweep_point_index
                        ],
                        output_keys=["look_point"],
                    ),
                    transitions={"succeeded": "LOOK_POINT"},
                    remapping={"look_point": "pointstamped"},
                )
                smach.StateMachine.add(
                    "LOOK_POINT",
                    LookToPoint(),
                    transitions={"succeeded": "DETECT_OBJECTS", "failed": "failed"},
                )
                smach.StateMachine.add(
                    "DETECT_OBJECTS",
                    Detect3DInArea(
                        area_polygon=self._polygon,
                        filter=self._object_filter,
                        confidence=self._min_confidence,
                    ),
                    transitions={"succeeded": "PROCESS_DETECTIONS", "failed": "failed"},
                    remapping={
                        "detections_3d": "detections_3d",
                        "image_raw": "image_raw",
                    },
                )
                smach.StateMachine.add(
                    "PROCESS_DETECTIONS",
                    ProcessDetections(min_new_object_dist=self._min_new_object_dist),
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
                "succeeded": "succeeded",
                "failed": "failed",
            },
            remapping={
                "sweep_points": "sweep_points",
                "detected_objects": "detected_objects",
            },
        )


if __name__ == "__main__":
    seat_area = [
        [2.02766489982605, -2.7318179607391357],
        [-0.8237523436546326, -2.8495190143585205],
        [-0.7675997614860535, -1.3231755495071411],
        [1.9695378541946411, -1.4235490560531616],
    ]
    seat_polygon = ShapelyPolygon(seat_area)
    rospy.init_node("detect_all_in_polygon")
    sm = DetectAllInPolygon(seat_polygon, object_filter=["person", "chair"])
    outcome = sm.execute()
    rospy.loginfo(f"State machine finished with outcome: {outcome}")
