import smach
import rospy

from typing import Union, List, Optional
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
            output_keys=["object_found", "object_name"],
        )

    def execute(self, userdata):
        rospy.loginfo("Processing detections...")
        if userdata.detected_objects:
            userdata.object_found = True
        else:
            userdata.object_found = False
            return "failed"
        found_object = userdata.detected_objects[0]
        userdata.object_name = found_object.name
        rospy.loginfo(f"Found object: {found_object.name}")
        return "succeeded"


class FindObjectAtLoc(smach.StateMachine):
    """
    State machine for finding an object at a given location,
    specified by a Shapely Polygon.
    """

    _object_name: Union[str, List[str]]
    _location_polygon: ShapelyPolygon
    _object_category: Optional[str]

    def __init__(
        self,
        object_name: Union[
            str, List[str]
        ],  # List as can be told to find a category of object
        location_polygon: Union[ShapelyPolygon, Polygon],
        object_category: Optional[str] = None,
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["object_found"],
        )

        self._object_name = (
            [object_name] if isinstance(object_name, str) else object_name
        )
        self._location_polygon = (
            location_polygon
            if isinstance(location_polygon, ShapelyPolygon)
            else ShapelyPolygon(
                [(point.x, point.y) for point in location_polygon.points]
            )
        )
        self._object_category = object_category

        with self:
            smach.StateMachine.add(
                "DetectObjectsInArea",
                DetectAllInPolygon(
                    polygon=self._location_polygon,
                    object_filter=self._object_name,
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
            if self._object_category:

                smach.StateMachine.add(
                    "SAY_OBJECT_FOUND",
                    Say(
                        format_str="I have found {} of category "
                        + self._object_category
                    ),
                    transitions={
                        "succeeded": "succeeded",
                        "preempted": "succeeded",
                        "aborted": "succeeded",
                    },
                    remapping={"placeholders": "object_name"},
                )
            else:
                smach.StateMachine.add(
                    "SAY_OBJECT_FOUND",
                    Say(f"I have found the {self._object_name}"),
                    transitions={
                        "succeeded": "succeeded",
                        "preempted": "succeeded",
                        "aborted": "succeeded",
                    },
                )
            smach.StateMachine.add(
                "SAY_OBJECT_NOT_FOUND",
                Say(f"I could not find the {self._object_name} here."),
                transitions={
                    "succeeded": "failed",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )
