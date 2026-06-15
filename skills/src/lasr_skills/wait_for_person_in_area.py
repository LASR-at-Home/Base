import rclpy
from typing import Union

import yasmin
import yasmin_ros
from yasmin import State, StateMachine

from lasr_skills import Detect3DInArea, Wait

from shapely import Polygon as ShapelyPolygon
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
    def __init__(
        self,
        polygon: Union[ShapelyPolygon, None] = None,
        polygon_param: Union[str, None] = None,
    ):
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)
        self.add_input_key("polygon")
        self.add_output_key("detections_3d")

        self.detection_polygon = None

        if polygon:
            self.detection_polygon = polygon
        elif polygon_param:
            node = yasmin_ros.logger_node

            top_left = rclpy.parameter.parameter_value_to_python(
                node.get_parameter(f"{polygon_param}.top_left").get_parameter_value()
            )
            top_right = rclpy.parameter.parameter_value_to_python(
                node.get_parameter(f"{polygon_param}.top_right").get_parameter_value()
            )
            bottom_left = rclpy.parameter.parameter_value_to_python(
                node.get_parameter(f"{polygon_param}.bottom_left").get_parameter_value()
            )
            bottom_right = rclpy.parameter.parameter_value_to_python(
                node.get_parameter(
                    f"{polygon_param}.bottom_right"
                ).get_parameter_value()
            )

            self.detection_polygon = ShapelyPolygon(
                [top_left, top_right, bottom_right, bottom_left]
            )

        self.add_state(
            "CHECK_BLACKBOARD_FOR_POLYGON",
            yasmin.CbState(outcomes=["succeeded", "failed"], callback=self.check),
            transitions={"succeeded": "DETECT_PEOPLE_3D", "failed": "failed"},
        )
        self.add_state(
            "DETECT_PEOPLE_3D",
            Detect3DInArea(
                area_polygon=self.detection_polygon,
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
    def check(self, blackboard):
        try:
            if "polygon" in blackboard.keys():  # Update polygon
                self.detection_polygon = blackboard["polygon"]
                return "succeeded"
            elif self.detection_polygon:  # If a polygon is already defined
                return "succeeded"
            else:  # No polygon defined
                return "failed"
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(
                f"The following error occured while retrieving waiting polygon: {e}"
            )
            return "failed"
