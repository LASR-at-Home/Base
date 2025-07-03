import smach
import rospy
from lasr_skills import DetectLangSam3D
from typing import Union

from geometry_msgs.msg import Polygon, Point, Point32, PolygonStamped
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon as ShapelyPolygon
from std_msgs.msg import Header


class Detect3DInArea(smach.StateMachine):
    class FilterDetections(smach.State):
        def __init__(
            self,
            area_polygon: ShapelyPolygon,
            debug_publisher: str = "/skills/detect3d_in_area/debug",
        ):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["lang_sam_detections_3d"],
                output_keys=["lang_sam_detections_3d"],
            )
            self.area_polygon = area_polygon
            self.debug_publisher = rospy.Publisher(
                debug_publisher, PolygonStamped, queue_size=1
            )

        def execute(self, userdata):
            detected_objects = userdata["lang_sam_detections_3d"].detections
            # publish polygon for debugging

            polygon_msg = Polygon()
            polygon_msg.points = [
                Point32(x=point[0], y=point[1], z=0.0)
                for point in self.area_polygon.exterior.coords
            ]
            self.debug_publisher.publish(
                PolygonStamped(polygon=polygon_msg, header=Header(frame_id="map"))
            )
            satisfied_points = [
                self.area_polygon.contains(Point(object.point.x, object.point.y))
                for object in detected_objects
            ]
            filtered_detections = [
                detected_objects[i]
                for i in range(0, len(detected_objects))
                if satisfied_points[i]
            ]
            # List of Detection3D msgs
            userdata["lang_sam_detections_3d"] = filtered_detections
            return "succeeded"

    def __init__(
        self,
        area_polygon: ShapelyPolygon,
        image_topic: str = "/xtion/rgb/image_raw",
        depth_image_topic: str = "/xtion/depth_registered/image_raw",
        depth_camera_info_topic: str = "/xtion/depth_registered/camera_info",
        target_frame: str = "map",
        box_threshold: float = 0.3,
        text_threshold: float = 0.3,
        prompt: Union[str, None] = None,
    ):
        if prompt is None:
            input_keys = ["lang_sam_prompt"]
        else:
            input_keys = []
        smach.StateMachine.__init__(
            self,
            input_keys=input_keys,
            outcomes=["succeeded", "failed"],
            output_keys=["lang_sam_detections_3d", "image_raw"],
        )

        with self:
            smach.StateMachine.add(
                "DETECT_OBJECTS_3D",
                DetectLangSam3D(
                    image_topic=image_topic,
                    depth_image_topic=depth_image_topic,
                    depth_camera_info_topic=depth_camera_info_topic,
                    target_frame=target_frame,
                    box_threshold=box_threshold,
                    text_threshold=text_threshold,
                    prompt=prompt,
                ),
                transitions={"succeeded": "FILTER_DETECTIONS", "failed": "failed"},
            )
            smach.StateMachine.add(
                "FILTER_DETECTIONS",
                self.FilterDetections(area_polygon),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
