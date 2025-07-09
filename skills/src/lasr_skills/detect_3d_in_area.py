import smach
import rospy
from lasr_skills import Detect3D
from typing import List, Union, Optional

from geometry_msgs.msg import Polygon, Point, Point32, PolygonStamped
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon as ShapelyPolygon
from std_msgs.msg import Header


class Detect3DInArea(smach.StateMachine):
    class FilterDetections(smach.State):
        def __init__(
            self,
            area_polygon: Optional[ShapelyPolygon] = None,
            debug_publisher: str = "/skills/detect3d_in_area/debug",
        ):
            input_keys = (
                ["detections_3d"]
                if area_polygon is None
                else ["detections_3d", "polygon"]
            )
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=input_keys,
                output_keys=["detections_3d"],
            )
            self.area_polygon = area_polygon
            self.debug_publisher = rospy.Publisher(
                debug_publisher, PolygonStamped, queue_size=1
            )

        def execute(self, userdata):
            detected_objects = userdata["detections_3d"].detected_objects
            # publish polygon for debugging

            polygon_msg = Polygon()
            if self.area_polygon is None:
                if "polygon" not in userdata:
                    rospy.logerr("No polygon provided in userdata.")
                    return "failed"
                area_polygon = userdata["polygon"]
            else:
                area_polygon = self.area_polygon
            polygon_msg.points = [
                Point32(x=point[0], y=point[1], z=0.0)
                for point in area_polygon.exterior.coords
            ]
            self.debug_publisher.publish(
                PolygonStamped(polygon=polygon_msg, header=Header(frame_id="map"))
            )
            satisfied_points = [
                area_polygon.contains(Point(object.point.x, object.point.y))
                for object in detected_objects
            ]
            filtered_detections = [
                detected_objects[i]
                for i in range(0, len(detected_objects))
                if satisfied_points[i]
            ]
            # List of Detection3D msgs
            userdata["detections_3d"] = filtered_detections
            return "succeeded"

    def __init__(
        self,
        area_polygon: Optional[ShapelyPolygon] = None,
        image_topic: str = "/xtion/rgb/image_raw",
        depth_image_topic: str = "/xtion/depth_registered/image_raw",
        depth_camera_info_topic: str = "/xtion/depth_registered/camera_info",
        point_cloud_topic: Optional[str] = None,
        model: str = "yolo11n-seg.pt",
        filter: Union[List[str], None] = None,
        confidence: float = 0.5,
        target_frame: str = "map",
    ):
        input_keys = ["polygon"] if area_polygon is None else []
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=input_keys,
            output_keys=["detections_3d", "image_raw", "pcl"],
        )

        with self:
            smach.StateMachine.add(
                "DETECT_OBJECTS_3D",
                Detect3D(
                    image_topic=image_topic,
                    depth_image_topic=depth_image_topic,
                    depth_camera_info_topic=depth_camera_info_topic,
                    point_cloud_topic=point_cloud_topic,
                    model=model,
                    filter=filter,
                    confidence=confidence,
                    target_frame=target_frame,
                ),
                transitions={"succeeded": "FILTER_DETECTIONS", "failed": "failed"},
            )
            smach.StateMachine.add(
                "FILTER_DETECTIONS",
                self.FilterDetections(area_polygon),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
