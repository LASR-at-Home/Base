import smach
import rospy

from lasr_skills import DetectAllInPolygon
from shapely.geometry import Polygon as ShapelyPolygon


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
        location_polygon: ShapelyPolygon,
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["object_found"],
        )

        self._object_name = object_name
        self._location_polygon = location_polygon

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
