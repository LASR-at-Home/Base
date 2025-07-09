from typing import List

import smach
import rospy

from lasr_skills import DetectAllInPolygon, Say, LookToPoint
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon as ShapelyPolygon
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header


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
        if drink_location == "None" or drink_location == "":
            drink_str += "Unfortunately, I couldn't find your drink on the table. "
        else:
            drink_str += f"Your drink is located at the {drink_location} of the table. "
        userdata.drink_string = drink_str
        return "succeeded"


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
            input_keys=["guest_data", "drink_detections"],
            output_keys=["drink_detections"],
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
                    "detected_objects": "detected_objects",
                    "guest_data": "guest_data",
                    "drink_location": "drink_location",
                },
            )
            smach.StateMachine.add(
                "GET_DRINK_STRING",
                GetDrinkString(),
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
                input_keys=["detected_objects", "guest_data", "drink_detections"],
                output_keys=[
                    "drink_location",  # Left, Centre, Right, or None
                    "drink_detections",
                ],
            )
            self._guest_id = guest_id
            self._table_left_area = table_left_area
            self._table_right_area = table_right_area
            self._table_centre_area = table_centre_area

        def execute(self, userdata):
            favourite_drink = userdata.guest_data[self._guest_id].get("drink", None)
            favourite_drink_location = "None"

            detected_drinks = userdata.detected_objects
            for drink in detected_drinks:
                point = Point(drink.point.x, drink.point.y)
                if self._table_left_area.contains(point):
                    drink_location = "Left"
                elif self._table_right_area.contains(point):
                    drink_location = "Right"
                elif self._table_centre_area.contains(point):
                    drink_location = "Centre"
                else:
                    drink_location = "None"
                if drink.name.lower() == favourite_drink.lower():
                    favourite_drink_location = drink_location

                userdata.drink_detections[drink.name.lower()]["detected"] = True
                userdata.drink_detections[drink.name.lower()][
                    "location"
                ] = drink_location

            rospy.loginfo(f"Detected drinks: {userdata.drink_detections}")

            userdata.drink_location = favourite_drink_location
            return "succeeded"


class GetDrinkLocationFromMemory(smach.State):
    def __init__(self, guest_id: str):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["drink_detections", "guest_data"],
            output_keys=["drink_location"],
        )

        self._guest_id = guest_id

    def execute(self, userdata):
        favourite_drink = userdata.guest_data[self._guest_id].get("drink", None)
        favourite_drink_location = "None"

        if favourite_drink is not None:
            try:
                if userdata.drink_detections[favourite_drink.lower()]["detected"]:
                    favourite_drink_location = userdata.drink_detections[
                        favourite_drink.lower()
                    ]["location"]
            except:
                return "failed"
        userdata.drink_location = favourite_drink_location
        return "succeeded"


class RecallDrinkOnTable(smach.StateMachine):
    def __init__(self, guest_id: str):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data", "drink_detections"],
        )

        with self:
            smach.StateMachine.add(
                "GET_DRINK_LOCATION_FROM_MEMORY",
                GetDrinkLocationFromMemory(guest_id=guest_id),
                transitions={
                    "succeeded": "GET_DRINK_STRING",
                    "failed": "GET_DRINK_STRING",
                },
                remapping={
                    "guest_data": "guest_data",
                    "drink_detections": "drink_detections",
                    "drink_location": "drink_location",
                },
            )
            smach.StateMachine.add(
                "GET_DRINK_STRING",
                GetDrinkString(),
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
