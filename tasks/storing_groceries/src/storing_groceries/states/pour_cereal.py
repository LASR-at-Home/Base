import smach


from lasr_skills import DetectAllInPolygonSensorData
from shapely import Polygon as ShapelyPolygon
import rospy

class PourCereal(smach.StateMachine):
    """
    State machine for pouring cereal into a bowl.
    1. detect the container and cereal
    2. grasp the cereal box
    3. move the cereal box to the container location by adding a offset of height
    4. pouring the cereal by flipping the orientation of the pose

    """

    smach.StateMachine.add(
        "DETECT_CEREAL",
        DetectAllInPolygonSensorData(
            ShapelyPolygon(rospy.get_param("/storing_groceries/table/polygon")),
            object_filter=['cereal'],
            min_coverage=0.9,
            min_confidence=0.1,
            z_axis=0.8,
            model="lasr.pt",
        ),
        transitions={"succeeded": "DETECT_CONTAINER", "failed": "failed"},
    )

    smach.StateMachine.add(
        "DETECT_CONTAINER",
        DetectAllInPolygonSensorData(
            ShapelyPolygon(rospy.get_param("/storing_groceries/table/polygon")),
            object_filter=['container'],
            min_coverage=0.9,
            min_confidence=0.1,
            z_axis=0.8,
            model="lasr.pt",
        ),
        transitions={"succeeded": "SELECT_OBJECT", "failed": "failed"},
    )



