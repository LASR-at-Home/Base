#!/usr/bin/env python3
import os
import smach
import rospy
import rospkg
import sys
from typing import Dict, List
from gpsr.load_known_data import GPSRDataLoader
from gpsr.state_machine_factory import build_state_machine
from gpsr.regex_command_parser import Configuration
from gpsr.states import CommandParserStateMachine, Survey, ChooseWavingPerson, ComputeApproach
from gpsr.regex_command_parser import gpsr_compile_and_parse

import actionlib
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Polygon
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pal_interaction_msgs.msg import TtsGoal, TtsAction
from shapely.geometry import Polygon as ShapelyPolygon

from lasr_skills import (
    GoToLocation,
    DetectDoorOpening,
    Say,
    ListenForWakeword,
    DetectAllInPolygon,
)


def load_gpsr_configuration() -> Configuration:
    gpsr_data_dir = os.path.join(
        rospkg.RosPack().get_path("gpsr"), "data", "salvador_data"
    )
    """Loads the configuration for the GPSR command parser"""
    data_loader = GPSRDataLoader(data_dir=gpsr_data_dir)
    gpsr_known_data: Dict = data_loader.load_data()
    config = Configuration(
        {
            "person_names": gpsr_known_data["names"],
            "location_names": gpsr_known_data["non_placeable_locations"],
            "placement_location_names": gpsr_known_data["placeable_locations"],
            "room_names": gpsr_known_data["rooms"],
            "object_names": gpsr_known_data["objects"],
            "object_categories_plural": gpsr_known_data["categories_plural"],
            "object_categories_singular": gpsr_known_data["categories_singular"],
        }
    )
    return config


def _move_base(client: actionlib.SimpleActionClient, pose: PoseStamped) -> MoveBaseGoal:
    goal: MoveBaseGoal = MoveBaseGoal()
    goal.target_pose = pose
    client.send_goal(goal)
    client.wait_for_result()

    return goal


def _tts(client: actionlib.SimpleActionClient, text: str) -> None:
    tts_goal: TtsGoal = TtsGoal()
    tts_goal.rawtext.text = text
    tts_goal.rawtext.lang_id = "en_GB"
    client.send_goal(tts_goal)
    client.wait_for_result()


class Start(smach.StateMachine):

    def __init__(self):

        super().__init__(outcomes=["succeeded"])

        with self:

            smach.StateMachine.add(
                "WAIT_FOR_DOOR_TO_OPEN",
                DetectDoorOpening(),
                transitions={"door_opened": "succeeded"},
            )


class GoToInstructionPoint(smach.StateMachine):

    def __init__(self):

        super().__init__(outcomes=["succeeded", "failed"])

        instruction_pose_param = rospy.get_param("gpsr/arena/start_pose")
        instruction_pose = Pose(
            position=Point(**instruction_pose_param["position"]),
            orientation=Quaternion(**instruction_pose_param["orientation"]),
        )
        with self:
            smach.StateMachine.add(
                "SAY_GOING_TO_START",
                Say(text="I am going to the instruction point to receive a command."),
                transitions={
                    "succeeded": "GO_TO_START",
                    "preempted": "GO_TO_START",
                    "aborted": "GO_TO_START",
                },
            )

            smach.StateMachine.add(
                "GO_TO_START",
                GoToLocation(location=instruction_pose),
                transitions={"succeeded": "succeeded", "failed": "GO_TO_START"},
            )


class WaitForWakeword(smach.StateMachine):

    def __init__(self, timeout: float):

        super().__init__(outcomes=["succeeded"])

        with self:

            smach.StateMachine.add(
                "SAY_HI_TIAGO",
                Say(
                    text="Please say 'hi tiago' when you are ready to give me your command."
                ),
                transitions={
                    "succeeded": "LISTEN_FOR_HI_TIAGO",
                    "preempted": "LISTEN_FOR_HI_TIAGO",
                    "aborted": "LISTEN_FOR_HI_TIAGO",
                },
            )

            smach.StateMachine.add(
                "LISTEN_FOR_HI_TIAGO",
                ListenForWakeword(wakeword="hi_tiago", timeout=timeout, threshold=0.01),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "succeeded",
                },
            )


def get_location_room(location: str) -> str:
    rooms = rospy.get_param("/gpsr/arena/rooms")
    for room in rooms:
        if location in rooms[room]["beacons"]:
            return room
    raise ValueError(f"Location {location} not found in the arena")


def get_location_pose(location: str, person: bool) -> Pose:
    location_room = get_location_room(location)
    if person:
        location_pose: Dict = rospy.get_param(
            f"/gpsr/arena/rooms/{location_room}/beacons/{location}/person_detection_pose"
        )
    else:
        location_pose: Dict = rospy.get_param(
            f"/gpsr/arena/rooms/{location_room}/beacons/{location}/object_detection_pose"
        )

    return Pose(
        position=Point(**location_pose["position"]),
        orientation=Quaternion(**location_pose["orientation"]),
    )


def get_location_polygon(location: str) -> Polygon:
    location_room = get_location_room(location)
    if "object_detection_polygon" in rospy.get_param(
        f"/gpsr/arena/rooms/{location_room}/beacons/{location}"
    ):
        polygon_points = rospy.get_param(
            f"/gpsr/arena/rooms/{location_room}/beacons/{location}/object_detection_polygon"
        )
        return Polygon([Point(p[0], p[1], 0.0) for p in polygon_points])
    else:
        raise ValueError(
            f"Location {location} does not have an object detection polygon."
        )


class PatrolObjectLocations(smach.StateMachine):

    class ProcessDetectionsInLocation(smach.State):

        def __init__(self, expected_object_category: str):
            self._object_categories = {
                "food": ["mayo", "tuna", "ketchup", "oats", "corn_flour", "broth"],
                "foods": ["mayo", "tuna", "ketchup", "oats", "corn_flour", "broth"],
                "snack": [
                    "peanuts",
                    "cornflakes",
                    "crisps",
                    "pringles",
                    "cheese_snack",
                    "chocolate_bar",
                    "gum_balls",
                ],
                "snacks": [
                    "peanuts",
                    "cornflakes",
                    "crisps",
                    "pringles",
                    "cheese_snack",
                    "chocolate_bar",
                    "gum_balls",
                ],
                "fruit": ["apple", "lemon", "lime", "tangerine", "pear"],
                "fruits": ["apple", "lemon", "lime", "tangerine", "pear"],
                "dish": ["spoon", "plate", "cup", "fork", "bowl", "knife"],
                "dishes": ["spoon", "plate", "cup", "fork", "bowl", "knife"],
                "cleaning_supply": ["cloth", "polish", "brush", "sponge"],
                "cleaning_supplies": ["cloth", "polish", "brush", "sponge"],
                "drink": ["coffee", "kuat", "milk", "orange_juice", "fanta", "coke"],
                "drinks": ["coffee", "kuat", "milk", "orange_juice", "fanta", "coke"],
            }
            self._expected_object_category = expected_object_category
            self._object_category_locations = {
                "snack": "side_table",
                "snacks": "side_table",
                "dish": "kitchen_table",
                "dishes": "kitchen_table",
                "cleaning_supply": "dishwasher",
                "cleaning_supplies": "dishwasher",
                "fruit": "desk",
                "fruits": "desk",
                "drink": "bar",
                "drinks": "bar",
                "food": "cabinet",
                "foods": "cabinet",
            }

            super().__init__(
                outcomes=["succeeded"],
                input_keys=["detected_objects"],
                output_keys=["text"],  # str
            )

        def execute(self, userdata: smach.UserData) -> str:
            detections = userdata.detections
            if not detections:
                rospy.logwarn("No detections found.")
                userdata.text = "I couldn't detect any objects"
                return "succeeded"

            object_violations: List[str] = []
            violation_categories: List[str] = []
            detected_objects: List[str] = [
                detection.name.lower() for detection in detections
            ]
            for detection in detections:
                detection_name = detection.name.lower()
                # Get detection category from self._object_categories
                detection_category = None
                for category, objects in self._object_categories.items():
                    if detection_name in objects:
                        detection_category = category
                        break
                if detection_category is None:
                    rospy.logwarn(
                        f"Detection {detection_name} does not match any known object categories."
                    )
                    continue
                if detection_category != self._expected_object_category:
                    object_violations.append(detection_name)
                    violation_categories.append(detection_category)

            if object_violations:
                text = ""
                for violation, category in zip(object_violations, violation_categories):
                    correct_location = ""
                    for (
                        expected_category,
                        location,
                    ) in self._object_category_locations.items():
                        if category in self._object_categories[expected_category]:
                            correct_location = location
                            break
                    text += f"I found a {violation} which is not a {self._expected_object_category} as it is a {category}. It should be moved to the {correct_location}. "

                text += "I am unable to move them myself, please move them to the correct location that I stated."
                userdata.text = text
                return "succeeded"

            userdata.text = f"I found the objects {', '.join(detected_objects)} which are all of the correct category {self._expected_object_category}. I will now continue my patrol."
            return "succeeded"

    def __init__(self):

        super().__init__(outcomes=["succeeded", "failed"])

        # Patrol object locations
        # - Cabinet (living room, foods)
        # - Desk (office, fruits)
        # - Bar (office, drinks)
        # - Side Table (bedroom, snacks)
        # - Kitchen Table (kitchen, Dishes)
        # - Dishwasher (kitchen, cleaning supplies)

        locations = {}
        locations["cabinet"] = (
            {
                "object_category": "foods",
                "detection_pose": get_location_pose("cabinet", person=False),
                "detection_polygon": get_location_polygon("cabinet"),
            },
        )
        locations["desk"] = (
            {
                "object_category": "fruits",
                "detection_pose": get_location_pose("desk", person=False),
                "detection_polygon": get_location_polygon("desk"),
            },
        )
        locations["bar"] = (
            {
                "object_category": "drinks",
                "detection_pose": get_location_pose("bar", person=False),
                "detection_polygon": get_location_polygon("bar"),
            },
        )
        locations["side_table"] = (
            {
                "object_category": "snacks",
                "detection_pose": get_location_pose("side_table", person=False),
                "detection_polygon": get_location_polygon("side_table"),
            },
        )
        locations["kitchen_table"] = (
            {
                "object_category": "dishes",
                "detection_pose": get_location_pose("kitchen_table", person=False),
                "detection_polygon": get_location_polygon("kitchen_table"),
            },
        )
        locations["dishwasher"] = {
            "object_category": "cleaning supplies",
            "detection_pose": get_location_pose("dishwasher", person=False),
            "detection_polygon": get_location_polygon("dishwasher"),
        }

        office_pose = Pose()
        living_room_pose = Pose()
        kitchen_pose = Pose()
        bedroom_pose = Pose()

        with self:
            location_pose = locations["cabinet"]["detection_pose"]
            object_category = locations["cabinet"]["object_category"]
            detection_polygon = locations["cabinet"]["detection_polygon"]
            detection_polygon = (
                detection_polygon
                if isinstance(detection_polygon, ShapelyPolygon)
                else ShapelyPolygon(
                    [(point.x, point.y) for point in detection_polygon.points]
                )
            )

            smach.StateMachine.add(
                "SAY_GOING_TO_CABINET",
                Say(text=f"I am going to the cabinet to check for {object_category}."),
                transitions={
                    "succeeded": "GO_TO_CABINET",
                    "preempted": "GO_TO_CABINET",
                    "aborted": "GO_TO_CABINET",
                },
            )

            smach.StateMachine.add(
                "GO_TO_CABINET",
                GoToLocation(location=location_pose),
                transitions={
                    "succeeded": "DETECT_OBJECTS_IN_CABINET",
                    "failed": "GO_TO_CABINET",
                },
            )

            smach.StateMachine.add(
                "DETECT_OBJECTS_IN_CABINET",
                DetectAllInPolygon(
                    polygon=detection_polygon,
                    min_coverage=1.0,
                ),
                transitions={
                    "succeeded": "PROCESS_DETECTIONS_IN_CABINET",
                    "failed": "SAY_GOING_TO_LIVING_ROOM",
                },
                remapping={"detected_objects": "detected_objects"},
            )

            smach.StateMachine.add(
                "PROCESS_DETECTIONS_IN_CABINET",
                self.ProcessDetectionsInLocation(
                    expected_object_category=object_category
                ),
                transitions={
                    "succeeded": "SAY_DONE_WITH_CABINET",
                },
                remapping={
                    "detected_objects": "detected_objects",
                    "text": "text",
                },
            )

            smach.StateMachine.add(
                "SAY_DONE_WITH_CABINET",
                Say(),
                transitions={
                    "succeeded": "SAY_GOING_TO_LIVING_ROOM",
                    "preempted": "SAY_GOING_TO_LIVING_ROOM",
                    "aborted": "SAY_GOING_TO_LIVING_ROOM",
                },
                remapping={
                    "text": "text",
                },
            )
            
            smach.StateMachine.add(
                "SAY_GOING_TO_LIVING_ROOM",
                Say(text="I am going to the living room to check for people that need help."),
                transitions={
                    "succeeded": "GO_TO_LIVING_ROOM",
                    "preempted": "GO_TO_LIVING_ROOM",
                    "aborted": "GO_TO_LIVING_ROOM",
                },
            )
            
            smach.StateMachine.add(
                "GO_TO_LIVING_ROOM",
                GoToLocation(location=living_room_pose),
                transitions={
                    "succeeded": "SURVEY_LIVING_ROOM",
                    "failed": "GO_TO_LIVING_ROOM",
                },
            )
            
            smach.StateMachine.add(
                "SURVEY_LIVING_ROOM",
                Survey(
                    look_range_deg=(-71.0, 71.0),
                    n_look_points=10,
                )
                transitions={
                    "customer_found": "SAY_CUSTOMER_FOUND",
                    "customer_not_found": "SAY_CUSTOMER_NOT_FOUND",
                },
                remapping={"hands_up_detections": "hands_up_detections"},
            )
            
            smach.StateMachine.add(
                "CHOOSE_WAVING_PERSON",
                ChooseWavingPerson(),
                transitions={
                    "succeeded": "COMPUTE_APPROACH_POSE",
                    "failed": "SAY_RAISE",
                },
                remapping={
                    "hands_up_detections": "hands_up_detections",
                    "waving_person_detection": "waving_person_detection",
                },
            )

            smach.StateMachine.add(
                "COMPUTE_APPROACH_POSE",
                ComputeApproach(),
                transitions={
                    "succeeded": "SAY_FOUND",
                    "failed": "SURVEY",
                },
                remapping={
                    "customer_approach_pose": "customer_approach_pose",
                },
            )

            smach.StateMachine.add(
                "SAY_FOUND",
                Say(
                    text="I have found a waving customer. I will navigate to them now."
                ),
                transitions={
                    "succeeded": "GO_TO_CUSTOMER",
                    "preempted": "GO_TO_BAR",
                    "aborted": "GO_TO_BAR",
                },
            )

            smach.StateMachine.add(
                "GO_TO_CUSTOMER",
                GoToLocation(),
                transitions={
                    "succeeded": "TAKE_ORDER",
                    "failed": "GO_TO_CUSTOMER",
                },
                remapping={
                    "location": "customer_approach_pose",
                },
            )
                

            location_pose = locations["desk"]["detection_pose"]
            object_category = locations["desk"]["object_category"]
            detection_polygon = locations["desk"]["detection_polygon"]
            detection_polygon = (
                detection_polygon
                if isinstance(detection_polygon, ShapelyPolygon)
                else ShapelyPolygon(
                    [(point.x, point.y) for point in detection_polygon.points]
                )
            )

            smach.StateMachine.add(
                "SAY_GOING_TO_DESK",
                Say(text=f"I am going to the desk to check for {object_category}."),
                transitions={
                    "succeeded": "GO_TO_DESK",
                    "preempted": "GO_TO_DESK",
                    "aborted": "GO_TO_DESK",
                },
            )

            smach.StateMachine.add(
                "GO_TO_DESK",
                GoToLocation(location=location_pose),
                transitions={
                    "succeeded": "DETECT_OBJECTS_IN_DESK",
                    "failed": "GO_TO_DESK",
                },
            )

            smach.StateMachine.add(
                "DETECT_OBJECTS_IN_DESK",
                DetectAllInPolygon(
                    polygon=detection_polygon,
                    min_coverage=1.0,
                ),
                transitions={
                    "succeeded": "PROCESS_DETECTIONS_IN_DESK",
                    "failed": "SAY_GOING_TO_BAR",
                },
                remapping={"detected_objects": "detected_objects"},
            )

            smach.StateMachine.add(
                "PROCESS_DETECTIONS_IN_DESK",
                self.ProcessDetectionsInLocation(
                    expected_object_category=object_category
                ),
                transitions={
                    "succeeded": "SAY_DONE_WITH_DESK",
                },
                remapping={
                    "detected_objects": "detected_objects",
                    "text": "text",
                },
            )

            smach.StateMachine.add(
                "SAY_DONE_WITH_DESK",
                Say(),
                transitions={
                    "succeeded": "SAY_GOING_TO_BAR",
                    "preempted": "SAY_GOING_TO_BAR",
                    "aborted": "SAY_GOING_TO_BAR",
                },
                remapping={
                    "text": "text",
                },
            )

            location_pose = locations["bar"]["detection_pose"]
            object_category = locations["bar"]["object_category"]
            detection_polygon = locations["bar"]["detection_polygon"]
            detection_polygon = (
                detection_polygon
                if isinstance(detection_polygon, ShapelyPolygon)
                else ShapelyPolygon(
                    [(point.x, point.y) for point in detection_polygon.points]
                )
            )

            smach.StateMachine.add(
                "SAY_GOING_TO_BAR",
                Say(text=f"I am going to the bar to check for {object_category}."),
                transitions={
                    "succeeded": "GO_TO_BAR",
                    "preempted": "GO_TO_BAR",
                    "aborted": "GO_TO_BAR",
                },
            )

            smach.StateMachine.add(
                "GO_TO_BAR",
                GoToLocation(location=location_pose),
                transitions={
                    "succeeded": "DETECT_OBJECTS_IN_BAR",
                    "failed": "GO_TO_BAR",
                },
            )

            smach.StateMachine.add(
                "DETECT_OBJECTS_IN_BAR",
                DetectAllInPolygon(
                    polygon=detection_polygon,
                    min_coverage=1.0,
                ),
                transitions={
                    "succeeded": "PROCESS_DETECTIONS_IN_BAR",
                    "failed": "SAY_GOING_TO_SIDE_TABLE",
                },
                remapping={"detected_objects": "detected_objects"},
            )

            smach.StateMachine.add(
                "PROCESS_DETECTIONS_IN_BAR",
                self.ProcessDetectionsInLocation(
                    expected_object_category=object_category
                ),
                transitions={
                    "succeeded": "SAY_DONE_WITH_BAR",
                },
                remapping={
                    "detected_objects": "detected_objects",
                    "text": "text",
                },
            )

            smach.StateMachine.add(
                "SAY_DONE_WITH_BAR",
                Say(),
                transitions={
                    "succeeded": "SAY_GOING_TO_SIDE_TABLE",
                    "preempted": "SAY_GOING_TO_SIDE_TABLE",
                    "aborted": "SAY_GOING_TO_SIDE_TABLE",
                },
                remapping={
                    "text": "text",
                },
            )


def main() -> None:
    N_COMMANDS: int = 3
    config = load_gpsr_configuration()
    tts_client = actionlib.SimpleActionClient("tts", TtsAction)
    tts_client.wait_for_server()
    _tts(tts_client, "Please open the door")
    timeout = 10.0
    start_sm = Start()
    start_sm.execute()
    rospy.sleep(3)

    # for i in range(N_COMMANDS):
    #     rospy.loginfo(f"Command {i + 1}")
    #     go_to_sm = GoToInstructionPoint()
    #     go_to_sm.execute()
    #     try:
    #         rospy.loginfo(f"Starting GPSR")
    #         wakeword_sm = WaitForWakeword(timeout)
    #         wakeword_sm.execute()
    #         timeout = 120.0
    #         command_parser_sm = CommandParserStateMachine(data_config=config)
    #         command_parser_sm.execute()
    #         parsed_command: Dict = command_parser_sm.userdata.parsed_command
    #         rospy.loginfo(f"Parsed command: {parsed_command}")
    #         sm = build_state_machine(parsed_command)
    #         sm.execute()
    #     except Exception as e:
    #         rospy.logerr(f"Error executing command: {e}")
    #         _tts(tts_client, "I am sorry, I couldn't understand your command")

    #         _tts(tts_client, "Something went wrong, I couldn't execute the command")
    # _tts(tts_client, "I am done")


if __name__ == "__main__":
    rospy.init_node("gpsr_main")
    main()
    rospy.spin()
