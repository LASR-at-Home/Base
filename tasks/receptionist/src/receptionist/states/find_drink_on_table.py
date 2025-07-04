from typing import List

import smach
import rospy

<<<<<<< HEAD
from lasr_skills import Detect3DInArea, Say, LookToPoint
=======
from lasr_skills import DetectAllInPolygon, Say, LookToPoint
>>>>>>> origin/main
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon as ShapelyPolygon
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header


class FindDrinkOnTable(smach.StateMachine):
    def __init__(
        self,
        guest_id: str,
        table_point: Point,
        table_area: ShapelyPolygon,
        table_left_area: ShapelyPolygon,
        table_right_area: ShapelyPolygon,
        table_centre_area: ShapelyPolygon,
        possible_drinks: List[str],
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data"],
        )
        self._guest_id = guest_id
        self.possible_drinks = possible_drinks

        with self:
            smach.StateMachine.add(
                "LOOK_AT_TABLE",
                LookToPoint(
                    pointstamped=PointStamped(
                        point=table_point, header=Header(frame_id="map")
                    )
                ),
                transitions={
                    "succeeded": "DETECT_DRINKS_ON_TABLE",
                    "aborted": "DETECT_DRINKS_ON_TABLE",
                    "timed_out": "DETECT_DRINKS_ON_TABLE",
                },
            )

            smach.StateMachine.add(
                "DETECT_DRINKS_ON_TABLE",
<<<<<<< HEAD
                Detect3DInArea(area_polygon=table_area, filter=self.possible_drinks),
                transitions={
                    "succeeded": "PROCESS_DETECTED_DRINKS",
                    "failed": "failed",
                },
            )
=======
                DetectAllInPolygon(
                    polygon=table_area,
                    object_filter=self.possible_drinks,
                    min_coverage=0.95,
                    min_new_object_dist=0.30,
                ),
                transitions={
                    "succeeded": "PROCESS_DETECTED_DRINKS",
                    "failed": "PROCESS_DETECTED_DRINKS",
                },
                remapping={
                    "detected_objects": "detected_objects",
                },
            )

>>>>>>> origin/main
            smach.StateMachine.add(
                "PROCESS_DETECTED_DRINKS",
                self.ProcessDetectedDrinks(
                    guest_id=self._guest_id,
                    table_left_area=table_left_area,
                    table_right_area=table_right_area,
                    table_centre_area=table_centre_area,
                ),
                transitions={
                    "succeeded": "GET_DRINK_STRING",
                    "failed": "GET_DRINK_STRING",
                },
                remapping={
<<<<<<< HEAD
                    "detections_3d": "detections_3d",
=======
                    "detected_objects": "detected_objects",
>>>>>>> origin/main
                    "guest_data": "guest_data",
                    "drink_location": "drink_location",
                },
            )
            smach.StateMachine.add(
                "GET_DRINK_STRING",
                self.GetDrinkString(),
                transitions={
                    "succeeded": "SAY_DRINK_STRING",
                    "failed": "SAY_DRINK_STRING",
                },
                remapping={
                    "guest_data": "guest_data",
                    "drink_location": "drink_location",
                    "drink_string": "drink_string",
                },
            )
            smach.StateMachine.add(
                "SAY_DRINK_STRING",
                Say(),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"text": "drink_string"},
            )

    class ProcessDetectedDrinks(smach.State):
        def __init__(
            self,
            guest_id: str,
            table_left_area: ShapelyPolygon,
            table_right_area: ShapelyPolygon,
            table_centre_area: ShapelyPolygon,
        ):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
<<<<<<< HEAD
                input_keys=["detections_3d", "guest_data"],
=======
                input_keys=["detected_objects", "guest_data"],
>>>>>>> origin/main
                output_keys=["drink_location"],  # Left, Centre, Right, or None
            )
            self._guest_id = guest_id
            self._table_left_area = table_left_area
            self._table_right_area = table_right_area
            self._table_centre_area = table_centre_area

        def execute(self, userdata):
            favourite_drink = userdata.guest_data[self._guest_id].get("drink", None)
            drink_location = "None"

            if favourite_drink is None:
                pass
            else:
<<<<<<< HEAD
                detected_drinks = userdata.detections_3d
=======
                detected_drinks = userdata.detected_objects
>>>>>>> origin/main
                for drink in detected_drinks:
                    if drink.name.lower() == favourite_drink.lower():
                        point = Point(drink.point.x, drink.point.y)
                        if self._table_left_area.contains(point):
                            drink_location = "Left"
                        elif self._table_right_area.contains(point):
                            drink_location = "Right"
                        elif self._table_centre_area.contains(point):
                            drink_location = "Centre"
                        else:
                            drink_location = "None"
                        break
            userdata.drink_location = drink_location
            return "succeeded"

    class GetDrinkString(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["guest_data", "drink_location"],
                output_keys=["drink_string"],
            )

        def execute(self, userdata):
            drink_location = userdata.drink_location
            drink_str = ""
            if drink_location == "None":
                drink_str += "Unfortunately, I couldn't find your drink on the table. "
            else:
                drink_str += (
                    f"Your drink is located at the {drink_location} of the table. "
                )
            userdata.drink_string = drink_str
            return "succeeded"
