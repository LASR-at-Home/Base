import smach
import rospy

from typing import Union
from lasr_skills import DetectAllInPolygon, Say
from shapely.geometry import Polygon as ShapelyPolygon
from geometry_msgs.msg import Polygon


class ProcessDetections(smach.State):
    """
    State to process the detected objects and check if the desired object is found.
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["detected_objects"],
            output_keys=["object_found"],
        )

    def execute(self, userdata):
        rospy.loginfo("Processing detections...")
        if userdata.detected_objects:
            userdata.object_found = True
            return "succeeded"
        else:
            userdata.object_found = False
            return "failed"


class FindObjectAtLoc(smach.StateMachine):
    """
    State machine for finding an object at a given location,
    specified by a Shapely Polygon.
    """

    _object_name: str
    _location_polygon: ShapelyPolygon

    def __init__(
        self,
        object_name: str,
        location_polygon: Union[ShapelyPolygon, Polygon],
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["object_found"],
        )

        self._object_name = object_name
        self._location_polygon = (
            location_polygon
            if isinstance(location_polygon, ShapelyPolygon)
            else ShapelyPolygon(
                [(point.x, point.y) for point in location_polygon.points]
            )
        )

        with self:
            smach.StateMachine.add(
                "DetectObjectsInArea",
                DetectAllInPolygon(
                    polygon=self._location_polygon,
                    object_filter=[self._object_name],
                    min_coverage=1.0,
                ),
                transitions={
                    "succeeded": "PROCESS_DETECTIONS",
                    "failed": "failed",
                },
                remapping={"detected_objects": "detected_objects"},
            )
            smach.StateMachine.add(
                "PROCESS_DETECTIONS",
                ProcessDetections(),
                transitions={
                    "succeeded": "SAY_OBJECT_FOUND",
                    "failed": "SAY_OBJECT_NOT_FOUND",
                },
                remapping={
                    "detected_objects": "detected_objects",
                    "object_found": "object_found",
                },
            )
            smach.StateMachine.add(
                "SAY_OBJECT_FOUND",
                Say(f"I have found the {self._object_name}"),
                transitions={"succeeded": "succeeded"},
            )
            smach.StateMachine.add(
                "SAY_OBJECT_NOT_FOUND",
                Say(f"I could not find the {self._object_name} here."),
                transitions={"succeeded": "failed"},
            )
