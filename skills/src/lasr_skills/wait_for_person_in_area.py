import rclpy

import yasmin_ros
from yasmin import State, StateMachine

from lasr_skills import Detect3DInArea

from shapely import Polygon as ShapelyPolygon


class CheckForPerson(State):
    def __init__(self):
        super().__init__(outcomes=["done", "not_done"])
        self.add_input_key("detections_3d")

    def execute(self, blackboard):
        if len(blackboard["detections_3d"]):
            return "done"
        else:
            return "not_done"


class WaitForPersonInArea(StateMachine):
    def __init__(self):
    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)
        self.add_output_key("detections_3d")

        node = yasmin_ros.logger_node

        top_left = rclpy.parameter.parameter_value_to_python(
            node.get_parameter("door_polygon.top_left").get_parameter_value()
        )
        top_right = rclpy.parameter.parameter_value_to_python(
            node.get_parameter("door_polygon.top_right").get_parameter_value()
        )
        bottom_left = rclpy.parameter.parameter_value_to_python(
            node.get_parameter("door_polygon.bottom_left").get_parameter_value()
        )
        bottom_right = rclpy.parameter.parameter_value_to_python(
            node.get_parameter("door_polygon.bottom_right").get_parameter_value()
        )

        door_polygon = ShapelyPolygon([top_left, top_right, bottom_right, bottom_left])
        door_polygon = ShapelyPolygon([top_left, top_right, bottom_right, bottom_left])

        self.add_state(
            "DETECT_PEOPLE_3D",
            Detect3DInArea(
                area_polygon=door_polygon,
                filter=["person"],
                z_min=-10,
                z_max=10.0,
            ),
            transitions={"succeeded": "CHECK_FOR_PERSON", "failed": "failed"},
        )
        self.add_state(
            "CHECK_FOR_PERSON",
            CheckForPerson(),
            transitions={"done": "succeeded", "not_done": "DETECT_PEOPLE_3D"},
        )
