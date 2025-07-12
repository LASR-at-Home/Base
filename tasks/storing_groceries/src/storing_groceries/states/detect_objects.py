import rospy
import smach
from shapely import Polygon as ShapelyPolygon


from lasr_skills import (
    DetectAllInPolygonSensorData,
    LookToPoint,
)

from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header


class DetectObjects(smach.StateMachine):

    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "failed"], output_keys=["detected_objects"]
        )

        with self:

            smach.StateMachine.add(
                "LOOK_AT_TABLE",
                LookToPoint(
                    pointstamped=PointStamped(
                        point=Point(
                            **rospy.get_param("/storing_groceries/table/look_point")
                        ),
                        header=Header(frame_id="map"),
                    )
                ),
                transitions={
                    "succeeded": "DETECT_OBJECTS",
                    "aborted": "DETECT_OBJECTS",
                    "timed_out": "DETECT_OBJECTS",
                },
            )

            smach.StateMachine.add(
                "DETECT_OBJECTS",
                DetectAllInPolygonSensorData(
                    ShapelyPolygon(rospy.get_param("/storing_groceries/table/polygon")),
                    object_filter=[
                        k for k in rospy.get_param("/storing_groceries/objects")
                    ],
                    min_coverage=0.9,
                    min_confidence=0.1,
                    z_axis=0.8,
                    model="lasr.pt",
                ),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
