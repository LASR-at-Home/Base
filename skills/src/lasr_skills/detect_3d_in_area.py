import rclpy
from rclpy.node import Node
from rclpy.time import Time

import yasmin
import yasmin_ros
from yasmin_ros import set_ros_loggers, ActionState

from .detect_3d import Detect3D
from typing import List, Union, Optional

from std_msgs.msg import Header
from geometry_msgs.msg import Polygon, Point, Point32, PolygonStamped, PointStamped
from shapely.geometry import Point as ShapelyPoint
from shapely.geometry.polygon import Polygon as ShapelyPolygon


class Detect3DInArea(yasmin.StateMachine):
    class FilterDetections(yasmin.State):
        def __init__(
            self,
            area_polygon: Optional[ShapelyPolygon] = None,
            z_min: Optional[float] = None,
            z_max: Optional[float] = None,
            debug_publisher: str = "/skills/detect3d_in_area/debug",
        ):
            super().__init__(outcomes=["succeeded", "failed"])

            self.add_input_key("detections_3d")
            if area_polygon is None:
                self.add_input_key("polygon")

            if z_min is None and z_max is None:
                self.add_input_key("z_sweep_min")
                self.add_input_key("z_sweep_max")

            self.add_output_key("detections_3d")

            self._z_min = z_min
            self._z_max = z_max
            self.area_polygon = area_polygon
            self.node = yasmin_ros.logger_node
            self.debug_publisher = self.node.create_publisher(
                PolygonStamped, debug_publisher, 1
            )

        def execute(self, blackboard):
            detected_objects = blackboard["detections_3d"].detected_objects
            # publish polygon for debugging
            if self._z_min is None:
                z_sweep_min = blackboard["z_sweep_min"]
            else:
                z_sweep_min = self._z_min
            if self._z_max is None:
                z_sweep_max = blackboard["z_sweep_max"]
            else:
                z_sweep_max = self._z_max
            polygon_msg = Polygon()
            if self.area_polygon is None:
                area_polygon = blackboard["polygon"]
            else:
                area_polygon = self.area_polygon

            polygon_msg.points = [
                Point32(x=point[0], y=point[1], z=0.0)
                for point in area_polygon.exterior.coords
            ]
            self.debug_publisher.publish(
                PolygonStamped(polygon=polygon_msg, header=Header(frame_id="map"))
            )

            pub = yasmin_ros.logger_node.create_publisher(  # CHECK:  New publisher each time? declare in __init__ instead?
                PointStamped, "objects_points", 10
            )

            for detection in detected_objects:
                if (
                    detection.point.x == "nan"
                ):  # CHECK:  Potential broken? float vs string?
                    yasmin.YASMIN_LOG_WARN(
                        "NAN detection check work"
                    )  # Remove line if works
                    continue
                yasmin.YASMIN_LOG_INFO(
                    f"Detected a {detection.name} at x:{detection.point.x}, y:{detection.point.y}, z:{detection.point.z}"
                )
                pub.publish(
                    PointStamped(
                        header=Header(
                            frame_id="map",
                            stamp=Time().to_msg(),
                        ),
                        point=Point(
                            x=detection.point.x,
                            y=detection.point.y,
                            z=detection.point.z,
                        ),
                    )
                )

            satisfied_points = [
                area_polygon.contains(ShapelyPoint(object.point.x, object.point.y))
                for object in detected_objects
            ]
            filtered_detections = [
                detected_objects[i]
                for i in range(0, len(detected_objects))
                if satisfied_points[i]
            ]
            filtered_detections = [
                detection
                for detection in filtered_detections
                if (detection.point.z >= z_sweep_min)
                and (detection.point.z <= z_sweep_max)
            ]
            # List of Detection3D msgs
            blackboard["detections_3d"] = filtered_detections
            return "succeeded"

    def __init__(
        self,
        area_polygon: Optional[ShapelyPolygon] = None,
        image_topic: str = "/head_front_camera/rgb/image_raw",
        depth_image_topic: str = "/head_front_camera/depth/image_raw",
        depth_camera_info_topic: str = "/head_front_camera/depth/camera_info",
        model: str = "yolo11n-seg.pt",
        filter: Union[List[str], None] = None,
        confidence: float = 0.5,
        target_frame: str = "map",
        z_min: Optional[float] = None,
        z_max: Optional[float] = None,
    ):

        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)
        if area_polygon is None:
            self.add_input_key("polygon")
        if z_min is None and z_max is None:
            self.add_input_key("z_sweep_min")
            self.add_input_key("z_sweep_max")

        self.add_output_key("detections_3d")
        self.add_output_key("image_raw")

        self.add_state(
            "DETECT_OBJECTS_3D",
            Detect3D(
                image_topic=image_topic,
                depth_image_topic=depth_image_topic,
                depth_camera_info_topic=depth_camera_info_topic,
                model=model,
                filter=filter,
                confidence=confidence,
                target_frame=target_frame,
            ),
            transitions={"succeeded": "FILTER_DETECTIONS", "failed": "failed"},
        )
        self.add_state(
            "FILTER_DETECTIONS",
            self.FilterDetections(area_polygon, z_min, z_max),
            transitions={"succeeded": "succeeded", "failed": "failed"},
        )
