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
from gpsr.states import (
    CommandParserStateMachine,
    Survey,
    ChooseWavingPerson,
    ComputeApproach,
)
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
import smach_ros
from std_msgs.msg import Empty


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
                "WAIT_START",
                smach_ros.MonitorState(
                    "/gpsr/start",
                    Empty,
                    lambda *_: False,
                ),
                transitions={
                    "valid": "WAIT_START",
                    "preempted": "WAIT_START",
                    "invalid": "SAY_WAITING",
                },
            )

            smach.StateMachine.add(
                "SAY_WAITING",
                Say(text="Waiting for the door to open."),
                transitions={
                    "succeeded": "WAIT_FOR_DOOR_TO_OPEN",
                    "aborted": "WAIT_FOR_DOOR_TO_OPEN",
                    "preempted": "WAIT_FOR_DOOR_TO_OPEN",
                },
            )

            smach.StateMachine.add(
                "WAIT_FOR_DOOR_TO_OPEN",
                DetectDoorOpening(timeout=10.0),
                transitions={
                    "door_opened": "succeeded",
                },
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

    class GetCommandString(smach.State):
        def __init__(self):
            super().__init__(
                outcomes=["succeeded", "failed"],
                input_keys=["parsed_command"],
                output_keys=["command_string"],
            )

        def execute(self, userdata: smach.UserData) -> str:
            try:
                sm, command_string = build_state_machine(userdata.parsed_command)
                userdata.command_string = command_string
                tts_client = actionlib.SimpleActionClient("tts", TtsAction)
                _tts(tts_client, command_string)
                sm.execute()
            except Exception as e:
                rospy.logerr(f"Error building state machine: {e}")
                command_string = "I wasn't able to understand your command, sorry."
                userdata.command_string = command_string
                try:
                    tts_client = actionlib.SimpleActionClient("tts", TtsAction)
                    _tts(tts_client, command_string)
                except:
                    return "failed"
                return "failed"
            return "succeeded"

    class ProcessDetectionsInLocation(smach.State):

        def __init__(self, expected_object_category: str):
            self._object_categories = {
                "food": ["mayo", "tuna", "ketchup", "oats", "corn_flour", "broth"],
                "snack": [
                    "peanuts",
                    "cornflakes",
                    "crisps",
                    "pringles",
                    "cheese_snack",
                    "chocolate_bar",
                    "gum_balls",
                ],
                "fruit": ["apple", "lemon", "lime", "tangerine", "pear"],
                "dish": ["spoon", "plate", "cup", "fork", "bowl", "knife"],
                "cleaning_supply": ["cloth", "polish", "brush", "sponge"],
                "drink": ["coffee", "kuat", "milk", "orange_juice", "fanta", "coke"],
            }
            self._expected_object_category = expected_object_category
            self._object_category_locations = {
                "snack": "side_table",
                "dish": "kitchen_table",
                "cleaning_supply": "dishwasher",
                "fruit": "desk",
                "drink": "bar",
                "food": "cabinet",
            }

            super().__init__(
                outcomes=["succeeded"],
                input_keys=["detected_objects"],
                output_keys=["text"],  # str
            )

        def execute(self, userdata: smach.UserData) -> str:
            detections = userdata.detected_objects
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
                        category,
                        location,
                    ) in self._object_category_locations.items():
                        if violation in self._object_categories[category]:
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
        # - TV Stand (living room, None)
        # - Sofa (living room, None)
        # - Desk (office, fruits)
        # - Bar (office, drinks)
        # - Side Table (bedroom, snacks)
        # - Bed (office, None)
        # - Bedside Table (bedroom, None)
        # - Kitchen Table (kitchen, Dishes)
        # - Dishwasher (kitchen, cleaning supplies)
        # - Sink (kitchen, None)
        # - Shelf (kitchen, None)
        # - Refrigerator (kitchen, None)

        locations = {}
        locations["cabinet"] = {
            "object_category": "food",
            "detection_pose": get_location_pose("cabinet", person=False),
            "detection_polygon": get_location_polygon("cabinet"),
        }
        locations["desk"] = {
            "object_category": "fruit",
            "detection_pose": get_location_pose("desk", person=False),
            "detection_polygon": get_location_polygon("desk"),
        }

        locations["bar"] = {
            "object_category": "drink",
            "detection_pose": get_location_pose("bar", person=False),
            "detection_polygon": get_location_polygon("bar"),
        }

        locations["side_table"] = {
            "object_category": "snack",
            "detection_pose": get_location_pose("side_table", person=False),
            "detection_polygon": get_location_polygon("side_table"),
        }
        locations["kitchen_table"] = {
            "object_category": "dish",
            "detection_pose": get_location_pose("kitchen_table", person=False),
            "detection_polygon": get_location_polygon("kitchen_table"),
        }
        locations["dishwasher"] = {
            "object_category": "cleaning_supply",
            "detection_pose": get_location_pose("dishwasher", person=False),
            "detection_polygon": get_location_polygon("dishwasher"),
        }
        locations["tv_stand"] = {
            "object_category": "None",
            "detection_pose": get_location_pose("tv_stand", person=False),
            "detection_polygon": get_location_polygon("tv_stand"),
        }
        locations["sofa"] = {
            "object_category": "None",
            "detection_pose": get_location_pose("sofa", person=False),
            "detection_polygon": get_location_polygon("sofa"),
        }
        locations["bed"] = {
            "object_category": "None",
            "detection_pose": get_location_pose("bed", person=False),
            "detection_polygon": get_location_polygon("bed"),
        }
        locations["bedside_table"] = {
            "object_category": "None",
            "detection_pose": get_location_pose("bedside_table", person=False),
            "detection_polygon": get_location_polygon("bedside_table"),
        }
        locations["sink"] = {
            "object_category": "None",
            "detection_pose": get_location_pose("sink", person=False),
            "detection_polygon": get_location_polygon("sink"),
        }
        locations["shelf"] = {
            "object_category": "None",
            "detection_pose": get_location_pose("shelf", person=False),
            "detection_polygon": get_location_polygon("shelf"),
        }
        locations["refrigerator"] = {
            "object_category": "None",
            "detection_pose": get_location_pose("refrigerator", person=False),
            "detection_polygon": get_location_polygon("refrigerator"),
        }

        office_pose = Pose(
            position=Point(6.545845364100279, 1.8987864086171355, 0.0),
            orientation=Quaternion(0.0, 0.0, -0.6584465340264694, 0.7526275053627323),
        )
        """OFFICE
        position: 
            x: 6.545845364100279
            y: 1.8987864086171355
            z: 0.0
        orientation: 
            x: 0.0
            y: 0.0
            z: -0.6584465340264694
            w: 0.7526275053627323
        
        """
        living_room_pose = Pose(
            position=Point(6.576560077776771, 1.8619710776864822, 0.0),
            orientation=Quaternion(0.0, 0.0, -0.9193216523479134, 0.3935069243663983),
        )
        """LIVING ROOM
        position: 
            x: 6.576560077776771
            y: 1.8619710776864822
            z: 0.0
        orientation: 
            x: 0.0
            y: 0.0
            z: -0.9193216523479134
            w: 0.3935069243663983
        """
        kitchen_pose = Pose(
            position=Point(6.457119880826167, 3.636448861465817, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.8818005401937303, 0.47162252630047835),
        )
        """KITCHEN POSE
        position: 
            x: 6.457119880826167
            y: 3.636448861465817
            z: 0.0
        orientation: 
            x: 0.0
            y: 0.0
            z: 0.8818005401937303
            w: 0.47162252630047835
        """
        bedroom_pose = Pose(
            position=Point(7.440938537594531, 2.9093494451150845, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.12446063864335499, 0.9922245458707863),
        )
        """BEDROOM
        position: 
            x: 7.440938537594531
            y: 2.9093494451150845
            z: 0.0
        orientation: 
            x: 0.0
            y: 0.0
            z: 0.12446063864335499
            w: 0.9922245458707863    
        """

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
                    "failed": "SAY_GOING_TO_TV_STAND",
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
                    "succeeded": "SAY_GOING_TO_TV_STAND",
                    "preempted": "SAY_GOING_TO_TV_STAND",
                    "aborted": "SAY_GOING_TO_TV_STAND",
                },
                remapping={
                    "text": "text",
                },
            )

            location_pose = locations["tv_stand"]["detection_pose"]
            object_category = locations["tv_stand"]["object_category"]
            detection_polygon = locations["tv_stand"]["detection_polygon"]
            detection_polygon = (
                detection_polygon
                if isinstance(detection_polygon, ShapelyPolygon)
                else ShapelyPolygon(
                    [(point.x, point.y) for point in detection_polygon.points]
                )
            )

            smach.StateMachine.add(
                "SAY_GOING_TO_TV_STAND",
                Say(text=f"I am going to the tv stand to check for {object_category}."),
                transitions={
                    "succeeded": "GO_TO_TV_STAND",
                    "preempted": "GO_TO_TV_STAND",
                    "aborted": "GO_TO_TV_STAND",
                },
            )

            smach.StateMachine.add(
                "GO_TO_TV_STAND",
                GoToLocation(location=location_pose),
                transitions={
                    "succeeded": "DETECT_OBJECTS_IN_TV_STAND",
                    "failed": "GO_TO_TV_STAND",
                },
            )

            smach.StateMachine.add(
                "DETECT_OBJECTS_IN_TV_STAND",
                DetectAllInPolygon(
                    polygon=detection_polygon,
                    min_coverage=1.0,
                ),
                transitions={
                    "succeeded": "PROCESS_DETECTIONS_IN_TV_STAND",
                    "failed": "SAY_GOING_TO_SOFA",
                },
                remapping={"detected_objects": "detected_objects"},
            )

            smach.StateMachine.add(
                "PROCESS_DETECTIONS_IN_TV_STAND",
                self.ProcessDetectionsInLocation(
                    expected_object_category=object_category
                ),
                transitions={
                    "succeeded": "SAY_DONE_WITH_TV_STAND",
                },
                remapping={
                    "detected_objects": "detected_objects",
                    "text": "text",
                },
            )

            smach.StateMachine.add(
                "SAY_DONE_WITH_TV_STAND",
                Say(),
                transitions={
                    "succeeded": "SAY_GOING_TO_SOFA",
                    "preempted": "SAY_GOING_TO_SOFA",
                    "aborted": "SAY_GOING_TO_SOFA",
                },
                remapping={
                    "text": "text",
                },
            )

            location_pose = locations["sofa"]["detection_pose"]
            object_category = locations["sofa"]["object_category"]
            detection_polygon = locations["sofa"]["detection_polygon"]
            detection_polygon = (
                detection_polygon
                if isinstance(detection_polygon, ShapelyPolygon)
                else ShapelyPolygon(
                    [(point.x, point.y) for point in detection_polygon.points]
                )
            )

            smach.StateMachine.add(
                "SAY_GOING_TO_SOFA",
                Say(text=f"I am going to the sofa to check for {object_category}."),
                transitions={
                    "succeeded": "GO_TO_SOFA",
                    "preempted": "GO_TO_SOFA",
                    "aborted": "GO_TO_SOFA",
                },
            )

            smach.StateMachine.add(
                "GO_TO_SOFA",
                GoToLocation(location=location_pose),
                transitions={
                    "succeeded": "DETECT_OBJECTS_IN_SOFA",
                    "failed": "GO_TO_SOFA",
                },
            )

            smach.StateMachine.add(
                "DETECT_OBJECTS_IN_SOFA",
                DetectAllInPolygon(
                    polygon=detection_polygon,
                    min_coverage=1.0,
                ),
                transitions={
                    "succeeded": "PROCESS_DETECTIONS_IN_SOFA",
                    "failed": "SAY_GOING_TO_LIVING_ROOM",
                },
                remapping={"detected_objects": "detected_objects"},
            )

            smach.StateMachine.add(
                "PROCESS_DETECTIONS_IN_SOFA",
                self.ProcessDetectionsInLocation(
                    expected_object_category=object_category
                ),
                transitions={
                    "succeeded": "SAY_DONE_WITH_SOFA",
                },
                remapping={
                    "detected_objects": "detected_objects",
                    "text": "text",
                },
            )

            smach.StateMachine.add(
                "SAY_DONE_WITH_SOFA",
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
                Say(
                    text="I am going to the living room to check for people that need help."
                ),
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
                    polygon=ShapelyPolygon(
                        rospy.get_param("/gpsr/arena/rooms/living_room/room_polygon")
                    ),
                ),
                transitions={
                    "customer_found": "SAY_PERSON_FOUND_LIVING_ROOM",
                    "customer_not_found": "SAY_PERSON_NOT_FOUND_LIVING_ROOM",
                },
                remapping={"hands_up_detections": "hands_up_detections"},
            )

            smach.StateMachine.add(
                "SAY_PERSON_FOUND_LIVING_ROOM",
                Say(text="I have found a person in the living room that needs help."),
                transitions={
                    "succeeded": "CHOOSE_WAVING_PERSON_LIVING_ROOM",
                    "preempted": "CHOOSE_WAVING_PERSON_LIVING_ROOM",
                    "aborted": "CHOOSE_WAVING_PERSON_LIVING_ROOM",
                },
                remapping={
                    "hands_up_detections": "hands_up_detections",
                },
            )

            smach.StateMachine.add(
                "SAY_PERSON_NOT_FOUND_LIVING_ROOM",
                Say(text="I can't see anyone in the living room that needs help."),
                transitions={
                    "succeeded": "SAY_GOING_TO_DESK",
                    "preempted": "SAY_GOING_TO_DESK",
                    "aborted": "SAY_GOING_TO_DESK",
                },
                remapping={
                    "hands_up_detections": "hands_up_detections",
                },
            )

            smach.StateMachine.add(
                "CHOOSE_WAVING_PERSON_LIVING_ROOM",
                ChooseWavingPerson(),
                transitions={
                    "succeeded": "COMPUTE_APPROACH_POSE_LIVING_ROOM",
                    "failed": "SAY_GOING_TO_DESK",
                },
                remapping={
                    "hands_up_detections": "hands_up_detections",
                    "waving_person_detection": "waving_person_detection",
                },
            )

            smach.StateMachine.add(
                "COMPUTE_APPROACH_POSE_LIVING_ROOM",
                ComputeApproach(),
                transitions={
                    "succeeded": "GO_TO_PERSON_LIVING_ROOM",
                    "failed": "SAY_GOING_TO_DESK",
                },
                remapping={
                    "customer_approach_pose": "customer_approach_pose",
                },
            )

            smach.StateMachine.add(
                "GO_TO_PERSON_LIVING_ROOM",
                GoToLocation(),
                transitions={
                    "succeeded": "ASK_FOR_COMMAND_LIVING_ROOM",
                    "failed": "GO_TO_PERSON_LIVING_ROOM",
                },
                remapping={
                    "location": "customer_approach_pose",
                },
            )

            smach.StateMachine.add(
                "ASK_FOR_COMMAND_LIVING_ROOM",
                CommandParserStateMachine(load_gpsr_configuration()),
                transitions={
                    "succeeded": "SAY_COMMAND_PLAN_LIVING_ROOM",
                    "failed": "RETRY_COMMAND_LIVING_ROOM_1",
                },
            )

            smach.StateMachine.add(
                "RETRY_COMMAND_LIVING_ROOM_1",
                CommandParserStateMachine(load_gpsr_configuration()),
                transitions={
                    "succeeded": "SAY_COMMAND_PLAN_LIVING_ROOM",
                    "failed": "RETRY_COMMAND_LIVING_ROOM_2",
                },
            )

            smach.StateMachine.add(
                "RETRY_COMMAND_LIVING_ROOM_2",
                CommandParserStateMachine(load_gpsr_configuration()),
                transitions={
                    "succeeded": "SAY_COMMAND_PLAN_LIVING_ROOM",
                    "failed": "SAY_GOING_TO_DESK",
                },
            )

            smach.StateMachine.add(
                "SAY_COMMAND_PLAN_LIVING_ROOM",
                self.GetCommandString(),
                transitions={
                    "succeeded": "SAY_GOING_TO_DESK",
                    "failed": "SAY_GOING_TO_DESK",
                },
                remapping={
                    "parsed_command": "parsed_command",
                    "command_string": "command_string",
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
                    "failed": "SAY_GOING_TO_OFFICE",
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
                    "succeeded": "SAY_GOING_TO_OFFICE",
                    "preempted": "SAY_GOING_TO_OFFICE",
                    "aborted": "SAY_GOING_TO_OFFICE",
                },
                remapping={
                    "text": "text",
                },
            )

            smach.StateMachine.add(
                "SAY_GOING_TO_OFFICE",
                Say(
                    text="I am going to the office to check for people that need help."
                ),
                transitions={
                    "succeeded": "GO_TO_OFFICE",
                    "preempted": "GO_TO_OFFICE",
                    "aborted": "GO_TO_OFFICE",
                },
            )

            smach.StateMachine.add(
                "GO_TO_OFFICE",
                GoToLocation(location=office_pose),
                transitions={
                    "succeeded": "SURVEY_OFFICE",
                    "failed": "GO_TO_OFFICE",
                },
            )

            smach.StateMachine.add(
                "SURVEY_OFFICE",
                Survey(
                    look_range_deg=(-71.0, 71.0),
                    n_look_points=10,
                    polygon=ShapelyPolygon(
                        rospy.get_param("/gpsr/arena/rooms/office/room_polygon")
                    ),
                ),
                transitions={
                    "customer_found": "SAY_PERSON_FOUND_OFFICE",
                    "customer_not_found": "SAY_PERSON_NOT_FOUND_OFFICE",
                },
                remapping={"hands_up_detections": "hands_up_detections"},
            )

            smach.StateMachine.add(
                "SAY_PERSON_FOUND_OFFICE",
                Say(text="I have found a person in the office that needs help."),
                transitions={
                    "succeeded": "CHOOSE_WAVING_PERSON_OFFICE",
                    "preempted": "CHOOSE_WAVING_PERSON_OFFICE",
                    "aborted": "CHOOSE_WAVING_PERSON_OFFICE",
                },
                remapping={
                    "hands_up_detections": "hands_up_detections",
                },
            )

            smach.StateMachine.add(
                "SAY_PERSON_NOT_FOUND_OFFICE",
                Say(text="I can't find anyone in the office that needs help."),
                transitions={
                    "succeeded": "SAY_GOING_TO_SIDE_TABLE",
                    "preempted": "SAY_GOING_TO_SIDE_TABLE",
                    "aborted": "SAY_GOING_TO_SIDE_TABLE",
                },
                remapping={
                    "hands_up_detections": "hands_up_detections",
                },
            )

            smach.StateMachine.add(
                "CHOOSE_WAVING_PERSON_OFFICE",
                ChooseWavingPerson(),
                transitions={
                    "succeeded": "COMPUTE_APPROACH_POSE_OFFICE",
                    "failed": "SAY_GOING_TO_SIDE_TABLE",
                },
                remapping={
                    "hands_up_detections": "hands_up_detections",
                    "waving_person_detection": "waving_person_detection",
                },
            )

            smach.StateMachine.add(
                "COMPUTE_APPROACH_POSE_OFFICE",
                ComputeApproach(),
                transitions={
                    "succeeded": "GO_TO_PERSON_OFFICE",
                    "failed": "SAY_GOING_TO_SIDE_TABLE",
                },
                remapping={
                    "customer_approach_pose": "customer_approach_pose",
                },
            )

            smach.StateMachine.add(
                "GO_TO_PERSON_OFFICE",
                GoToLocation(),
                transitions={
                    "succeeded": "ASK_FOR_COMMAND_OFFICE",
                    "failed": "GO_TO_PERSON_OFFICE",
                },
                remapping={
                    "location": "customer_approach_pose",
                },
            )

            smach.StateMachine.add(
                "ASK_FOR_COMMAND_OFFICE",
                CommandParserStateMachine(load_gpsr_configuration()),
                transitions={
                    "succeeded": "SAY_COMMAND_PLAN_OFFICE",
                    "failed": "RETRY_COMMAND_OFFICE_1",
                },
            )

            smach.StateMachine.add(
                "RETRY_COMMAND_OFFICE_1",
                CommandParserStateMachine(load_gpsr_configuration()),
                transitions={
                    "succeeded": "SAY_COMMAND_PLAN_OFFICE",
                    "failed": "RETRY_COMMAND_OFFICE_2",
                },
            )

            smach.StateMachine.add(
                "RETRY_COMMAND_OFFICE_2",
                CommandParserStateMachine(load_gpsr_configuration()),
                transitions={
                    "succeeded": "SAY_COMMAND_PLAN_OFFICE",
                    "failed": "SAY_GOING_TO_SIDE_TABLE",
                },
            )

            smach.StateMachine.add(
                "SAY_COMMAND_PLAN_OFFICE",
                self.GetCommandString(),
                transitions={
                    "succeeded": "SAY_GOING_TO_SIDE_TABLE",
                    "failed": "SAY_GOING_TO_SIDE_TABLE",
                },
                remapping={
                    "parsed_command": "parsed_command",
                    "command_string": "command_string",
                },
            )

            location_pose = locations["side_table"]["detection_pose"]
            object_category = locations["side_table"]["object_category"]
            detection_polygon = locations["side_table"]["detection_polygon"]
            detection_polygon = (
                detection_polygon
                if isinstance(detection_polygon, ShapelyPolygon)
                else ShapelyPolygon(
                    [(point.x, point.y) for point in detection_polygon.points]
                )
            )

            smach.StateMachine.add(
                "SAY_GOING_TO_SIDE_TABLE",
                Say(
                    text=f"I am going to the side table to check for {object_category}."
                ),
                transitions={
                    "succeeded": "GO_TO_SIDE_TABLE",
                    "preempted": "GO_TO_SIDE_TABLE",
                    "aborted": "GO_TO_SIDE_TABLE",
                },
            )

            smach.StateMachine.add(
                "GO_TO_SIDE_TABLE",
                GoToLocation(location=location_pose),
                transitions={
                    "succeeded": "DETECT_OBJECTS_IN_SIDE_TABLE",
                    "failed": "GO_TO_SIDE_TABLE",
                },
            )

            smach.StateMachine.add(
                "DETECT_OBJECTS_IN_SIDE_TABLE",
                DetectAllInPolygon(
                    polygon=detection_polygon,
                    min_coverage=1.0,
                ),
                transitions={
                    "succeeded": "PROCESS_DETECTIONS_IN_SIDE_TABLE",
                    "failed": "SAY_GOING_TO_BED",
                },
                remapping={"detected_objects": "detected_objects"},
            )

            smach.StateMachine.add(
                "PROCESS_DETECTIONS_IN_SIDE_TABLE",
                self.ProcessDetectionsInLocation(
                    expected_object_category=object_category
                ),
                transitions={
                    "succeeded": "SAY_DONE_WITH_SIDE_TABLE",
                },
                remapping={
                    "detected_objects": "detected_objects",
                    "text": "text",
                },
            )

            smach.StateMachine.add(
                "SAY_DONE_WITH_SIDE_TABLE",
                Say(),
                transitions={
                    "succeeded": "SAY_GOING_TO_BED",
                    "preempted": "SAY_GOING_TO_BED",
                    "aborted": "SAY_GOING_TO_BED",
                },
                remapping={
                    "text": "text",
                },
            )

            location_pose = locations["bed"]["detection_pose"]
            object_category = locations["bed"]["object_category"]
            detection_polygon = locations["bed"]["detection_polygon"]
            detection_polygon = (
                detection_polygon
                if isinstance(detection_polygon, ShapelyPolygon)
                else ShapelyPolygon(
                    [(point.x, point.y) for point in detection_polygon.points]
                )
            )

            smach.StateMachine.add(
                "SAY_GOING_TO_BED",
                Say(text=f"I am going to the bed to check for {object_category}."),
                transitions={
                    "succeeded": "GO_TO_BED",
                    "preempted": "GO_TO_BED",
                    "aborted": "GO_TO_BED",
                },
            )

            smach.StateMachine.add(
                "GO_TO_BED",
                GoToLocation(location=location_pose),
                transitions={
                    "succeeded": "DETECT_OBJECTS_IN_BED",
                    "failed": "GO_TO_BED",
                },
            )

            smach.StateMachine.add(
                "DETECT_OBJECTS_IN_BED",
                DetectAllInPolygon(
                    polygon=detection_polygon,
                    min_coverage=1.0,
                ),
                transitions={
                    "succeeded": "PROCESS_DETECTIONS_IN_BED",
                    "failed": "SAY_GOING_TO_BEDSIDE_TABLE",
                },
                remapping={"detected_objects": "detected_objects"},
            )

            smach.StateMachine.add(
                "PROCESS_DETECTIONS_IN_BED",
                self.ProcessDetectionsInLocation(
                    expected_object_category=object_category
                ),
                transitions={
                    "succeeded": "SAY_DONE_WITH_BED",
                },
                remapping={
                    "detected_objects": "detected_objects",
                    "text": "text",
                },
            )

            smach.StateMachine.add(
                "SAY_DONE_WITH_BED",
                Say(),
                transitions={
                    "succeeded": "SAY_GOING_TO_BEDSIDE_TABLE",
                    "preempted": "SAY_GOING_TO_BEDSIDE_TABLE",
                    "aborted": "SAY_GOING_TO_BEDSIDE_TABLE",
                },
                remapping={
                    "text": "text",
                },
            )

            location_pose = locations["bedside_table"]["detection_pose"]
            object_category = locations["bedside_table"]["object_category"]
            detection_polygon = locations["bedside_table"]["detection_polygon"]
            detection_polygon = (
                detection_polygon
                if isinstance(detection_polygon, ShapelyPolygon)
                else ShapelyPolygon(
                    [(point.x, point.y) for point in detection_polygon.points]
                )
            )

            smach.StateMachine.add(
                "SAY_GOING_TO_BEDSIDE_TABLE",
                Say(
                    text=f"I am going to the bedside table to check for {object_category}."
                ),
                transitions={
                    "succeeded": "GO_TO_BEDSIDE_TABLE",
                    "preempted": "GO_TO_BEDSIDE_TABLE",
                    "aborted": "GO_TO_BEDSIDE_TABLE",
                },
            )

            smach.StateMachine.add(
                "GO_TO_BEDSIDE_TABLE",
                GoToLocation(location=location_pose),
                transitions={
                    "succeeded": "DETECT_OBJECTS_IN_BEDSIDE_TABLE",
                    "failed": "GO_TO_BEDSIDE_TABLE",
                },
            )

            smach.StateMachine.add(
                "DETECT_OBJECTS_IN_BEDSIDE_TABLE",
                DetectAllInPolygon(
                    polygon=detection_polygon,
                    min_coverage=1.0,
                ),
                transitions={
                    "succeeded": "PROCESS_DETECTIONS_IN_BEDSIDE_TABLE",
                    "failed": "SAY_GOING_TO_BEDROOM",
                },
                remapping={"detected_objects": "detected_objects"},
            )

            smach.StateMachine.add(
                "PROCESS_DETECTIONS_IN_BEDSIDE_TABLE",
                self.ProcessDetectionsInLocation(
                    expected_object_category=object_category
                ),
                transitions={
                    "succeeded": "SAY_DONE_WITH_BEDSIDE_TABLE",
                },
                remapping={
                    "detected_objects": "detected_objects",
                    "text": "text",
                },
            )

            smach.StateMachine.add(
                "SAY_DONE_WITH_BEDSIDE_TABLE",
                Say(),
                transitions={
                    "succeeded": "SAY_GOING_TO_BEDROOM",
                    "preempted": "SAY_GOING_TO_BEDROOM",
                    "aborted": "SAY_GOING_TO_BEDROOM",
                },
                remapping={
                    "text": "text",
                },
            )

            smach.StateMachine.add(
                "SAY_GOING_TO_BEDROOM",
                Say(
                    text="I am going to the bedroom to check for people that need help."
                ),
                transitions={
                    "succeeded": "GO_TO_BEDROOM",
                    "preempted": "GO_TO_BEDROOM",
                    "aborted": "GO_TO_BEDROOM",
                },
            )

            smach.StateMachine.add(
                "GO_TO_BEDROOM",
                GoToLocation(location=bedroom_pose),
                transitions={
                    "succeeded": "SURVEY_BEDROOM",
                    "failed": "GO_TO_BEDROOM",
                },
            )

            smach.StateMachine.add(
                "SURVEY_BEDROOM",
                Survey(
                    look_range_deg=(-71.0, 71.0),
                    n_look_points=10,
                    polygon=ShapelyPolygon(
                        rospy.get_param("/gpsr/arena/rooms/bedroom/room_polygon")
                    ),
                ),
                transitions={
                    "customer_found": "SAY_PERSON_FOUND_BEDROOM",
                    "customer_not_found": "SAY_PERSON_NOT_FOUND_BEDROOM",
                },
                remapping={"hands_up_detections": "hands_up_detections"},
            )

            smach.StateMachine.add(
                "SAY_PERSON_FOUND_BEDROOM",
                Say(text="I have found a person in the bedroom that needs help."),
                transitions={
                    "succeeded": "CHOOSE_WAVING_PERSON_BEDROOM",
                    "preempted": "CHOOSE_WAVING_PERSON_BEDROOM",
                    "aborted": "CHOOSE_WAVING_PERSON_BEDROOM",
                },
                remapping={
                    "hands_up_detections": "hands_up_detections",
                },
            )

            smach.StateMachine.add(
                "SAY_PERSON_NOT_FOUND_BEDROOM",
                Say(text="I can't find anyone in the bedroom that needs help."),
                transitions={
                    "succeeded": "SAY_GOING_TO_DISHWASHER",
                    "preempted": "SAY_GOING_TO_DISHWASHER",
                    "aborted": "SAY_GOING_TO_DISHWASHER",
                },
                remapping={
                    "hands_up_detections": "hands_up_detections",
                },
            )

            smach.StateMachine.add(
                "CHOOSE_WAVING_PERSON_BEDROOM",
                ChooseWavingPerson(),
                transitions={
                    "succeeded": "COMPUTE_APPROACH_POSE_BEDROOM",
                    "failed": "SAY_GOING_TO_DISHWASHER",
                },
                remapping={
                    "hands_up_detections": "hands_up_detections",
                    "waving_person_detection": "waving_person_detection",
                },
            )

            smach.StateMachine.add(
                "COMPUTE_APPROACH_POSE_BEDROOM",
                ComputeApproach(),
                transitions={
                    "succeeded": "GO_TO_PERSON_BEDROOM",
                    "failed": "SAY_GOING_TO_DISHWASHER",
                },
                remapping={
                    "customer_approach_pose": "customer_approach_pose",
                },
            )

            smach.StateMachine.add(
                "GO_TO_PERSON_BEDROOM",
                GoToLocation(),
                transitions={
                    "succeeded": "ASK_FOR_COMMAND_BEDROOM",
                    "failed": "GO_TO_PERSON_BEDROOM",
                },
                remapping={
                    "location": "customer_approach_pose",
                },
            )

            smach.StateMachine.add(
                "ASK_FOR_COMMAND_BEDROOM",
                CommandParserStateMachine(load_gpsr_configuration()),
                transitions={
                    "succeeded": "SAY_COMMAND_PLAN_BEDROOM",
                    "failed": "RETRY_COMMAND_BEDROOM_1",
                },
            )

            smach.StateMachine.add(
                "RETRY_COMMAND_BEDROOM_1",
                CommandParserStateMachine(load_gpsr_configuration()),
                transitions={
                    "succeeded": "SAY_COMMAND_PLAN_BEDROOM",
                    "failed": "RETRY_COMMAND_BEDROOM_2",
                },
            )

            smach.StateMachine.add(
                "RETRY_COMMAND_BEDROOM_2",
                CommandParserStateMachine(load_gpsr_configuration()),
                transitions={
                    "succeeded": "SAY_COMMAND_PLAN_BEDROOM",
                    "failed": "SAY_GOING_TO_DISHWASHER",
                },
            )

            smach.StateMachine.add(
                "SAY_COMMAND_PLAN_BEDROOM",
                self.GetCommandString(),
                transitions={
                    "succeeded": "SAY_GOING_TO_DISHWASHER",
                    "failed": "SAY_GOING_TO_DISHWASHER",
                },
                remapping={
                    "parsed_command": "parsed_command",
                    "command_string": "command_string",
                },
            )

            location_pose = locations["dishwasher"]["detection_pose"]
            object_category = locations["dishwasher"]["object_category"]
            detection_polygon = locations["dishwasher"]["detection_polygon"]
            detection_polygon = (
                detection_polygon
                if isinstance(detection_polygon, ShapelyPolygon)
                else ShapelyPolygon(
                    [(point.x, point.y) for point in detection_polygon.points]
                )
            )

            smach.StateMachine.add(
                "SAY_GOING_TO_DISHWASHER",
                Say(
                    text=f"I am going to the dishwasher to check for {object_category}."
                ),
                transitions={
                    "succeeded": "GO_TO_DISHWASHER",
                    "preempted": "GO_TO_DISHWASHER",
                    "aborted": "GO_TO_DISHWASHER",
                },
            )

            smach.StateMachine.add(
                "GO_TO_DISHWASHER",
                GoToLocation(location=location_pose),
                transitions={
                    "succeeded": "DETECT_OBJECTS_IN_DISHWASHER",
                    "failed": "GO_TO_DISHWASHER",
                },
            )

            smach.StateMachine.add(
                "DETECT_OBJECTS_IN_DISHWASHER",
                DetectAllInPolygon(
                    polygon=detection_polygon,
                    min_coverage=1.0,
                ),
                transitions={
                    "succeeded": "PROCESS_DETECTIONS_IN_DISHWASHER",
                    "failed": "SAY_GOING_TO_KITCHEN_TABLE",
                },
                remapping={"detected_objects": "detected_objects"},
            )

            smach.StateMachine.add(
                "PROCESS_DETECTIONS_IN_DISHWASHER",
                self.ProcessDetectionsInLocation(
                    expected_object_category=object_category
                ),
                transitions={
                    "succeeded": "SAY_DONE_WITH_DISHWASHER",
                },
                remapping={
                    "detected_objects": "detected_objects",
                    "text": "text",
                },
            )

            smach.StateMachine.add(
                "SAY_DONE_WITH_DISHWASHER",
                Say(),
                transitions={
                    "succeeded": "SAY_GOING_TO_KITCHEN_TABLE",
                    "preempted": "SAY_GOING_TO_KITCHEN_TABLE",
                    "aborted": "SAY_GOING_TO_KITCHEN_TABLE",
                },
                remapping={
                    "text": "text",
                },
            )

            location_pose = locations["kitchen_table"]["detection_pose"]
            object_category = locations["kitchen_table"]["object_category"]
            detection_polygon = locations["kitchen_table"]["detection_polygon"]
            detection_polygon = (
                detection_polygon
                if isinstance(detection_polygon, ShapelyPolygon)
                else ShapelyPolygon(
                    [(point.x, point.y) for point in detection_polygon.points]
                )
            )

            smach.StateMachine.add(
                "SAY_GOING_TO_KITCHEN_TABLE",
                Say(
                    text=f"I am going to the kitchen table to check for {object_category}."
                ),
                transitions={
                    "succeeded": "GO_TO_KITCHEN_TABLE",
                    "preempted": "GO_TO_KITCHEN_TABLE",
                    "aborted": "GO_TO_KITCHEN_TABLE",
                },
            )

            smach.StateMachine.add(
                "GO_TO_KITCHEN_TABLE",
                GoToLocation(location=location_pose),
                transitions={
                    "succeeded": "DETECT_OBJECTS_IN_KITCHEN_TABLE",
                    "failed": "GO_TO_KITCHEN_TABLE",
                },
            )

            smach.StateMachine.add(
                "DETECT_OBJECTS_IN_KITCHEN_TABLE",
                DetectAllInPolygon(
                    polygon=detection_polygon,
                    min_coverage=1.0,
                ),
                transitions={
                    "succeeded": "PROCESS_DETECTIONS_IN_KITCHEN_TABLE",
                    "failed": "SAY_GOING_TO_SINK",
                },
                remapping={"detected_objects": "detected_objects"},
            )

            smach.StateMachine.add(
                "PROCESS_DETECTIONS_IN_KITCHEN_TABLE",
                self.ProcessDetectionsInLocation(
                    expected_object_category=object_category
                ),
                transitions={
                    "succeeded": "SAY_DONE_WITH_KITCHEN_TABLE",
                },
                remapping={
                    "detected_objects": "detected_objects",
                    "text": "text",
                },
            )

            smach.StateMachine.add(
                "SAY_DONE_WITH_KITCHEN_TABLE",
                Say(),
                transitions={
                    "succeeded": "SAY_GOING_TO_SINK",
                    "preempted": "SAY_GOING_TO_SINK",
                    "aborted": "SAY_GOING_TO_SINK",
                },
                remapping={
                    "text": "text",
                },
            )

            location_pose = locations["sink"]["detection_pose"]
            object_category = locations["sink"]["object_category"]
            detection_polygon = locations["sink"]["detection_polygon"]
            detection_polygon = (
                detection_polygon
                if isinstance(detection_polygon, ShapelyPolygon)
                else ShapelyPolygon(
                    [(point.x, point.y) for point in detection_polygon.points]
                )
            )

            smach.StateMachine.add(
                "SAY_GOING_TO_SINK",
                Say(text=f"I am going to the sink to check for {object_category}."),
                transitions={
                    "succeeded": "GO_TO_SINK",
                    "preempted": "GO_TO_SINK",
                    "aborted": "GO_TO_SINK",
                },
            )

            smach.StateMachine.add(
                "GO_TO_SINK",
                GoToLocation(location=location_pose),
                transitions={
                    "succeeded": "DETECT_OBJECTS_IN_SINK",
                    "failed": "GO_TO_SINK",
                },
            )

            smach.StateMachine.add(
                "DETECT_OBJECTS_IN_SINK",
                DetectAllInPolygon(
                    polygon=detection_polygon,
                    min_coverage=1.0,
                ),
                transitions={
                    "succeeded": "PROCESS_DETECTIONS_IN_SINK",
                    "failed": "SAY_GOING_TO_SHELF",
                },
                remapping={"detected_objects": "detected_objects"},
            )

            smach.StateMachine.add(
                "PROCESS_DETECTIONS_IN_SINK",
                self.ProcessDetectionsInLocation(
                    expected_object_category=object_category
                ),
                transitions={
                    "succeeded": "SAY_DONE_WITH_SINK",
                },
                remapping={
                    "detected_objects": "detected_objects",
                    "text": "text",
                },
            )

            smach.StateMachine.add(
                "SAY_DONE_WITH_SINK",
                Say(),
                transitions={
                    "succeeded": "SAY_GOING_TO_SHELF",
                    "preempted": "SAY_GOING_TO_SHELF",
                    "aborted": "SAY_GOING_TO_SHELF",
                },
                remapping={
                    "text": "text",
                },
            )

            location_pose = locations["shelf"]["detection_pose"]
            object_category = locations["shelf"]["object_category"]
            detection_polygon = locations["shelf"]["detection_polygon"]
            detection_polygon = (
                detection_polygon
                if isinstance(detection_polygon, ShapelyPolygon)
                else ShapelyPolygon(
                    [(point.x, point.y) for point in detection_polygon.points]
                )
            )

            smach.StateMachine.add(
                "SAY_GOING_TO_SHELF",
                Say(text=f"I am going to the shelf to check for {object_category}."),
                transitions={
                    "succeeded": "GO_TO_SHELF",
                    "preempted": "GO_TO_SHELF",
                    "aborted": "GO_TO_SHELF",
                },
            )

            smach.StateMachine.add(
                "GO_TO_SHELF",
                GoToLocation(location=location_pose),
                transitions={
                    "succeeded": "DETECT_OBJECTS_IN_SHELF",
                    "failed": "GO_TO_SHELF",
                },
            )

            smach.StateMachine.add(
                "DETECT_OBJECTS_IN_SHELF",
                DetectAllInPolygon(
                    polygon=detection_polygon,
                    min_coverage=1.0,
                ),
                transitions={
                    "succeeded": "PROCESS_DETECTIONS_IN_SHELF",
                    "failed": "SAY_GOING_TO_REFRIGERATOR",
                },
                remapping={"detected_objects": "detected_objects"},
            )

            smach.StateMachine.add(
                "PROCESS_DETECTIONS_IN_SHELF",
                self.ProcessDetectionsInLocation(
                    expected_object_category=object_category
                ),
                transitions={
                    "succeeded": "SAY_DONE_WITH_SHELF",
                },
                remapping={
                    "detected_objects": "detected_objects",
                    "text": "text",
                },
            )

            smach.StateMachine.add(
                "SAY_DONE_WITH_SHELF",
                Say(),
                transitions={
                    "succeeded": "SAY_GOING_TO_REFRIGERATOR",
                    "preempted": "SAY_GOING_TO_REFRIGERATOR",
                    "aborted": "SAY_GOING_TO_REFRIGERATOR",
                },
                remapping={
                    "text": "text",
                },
            )

            location_pose = locations["refrigerator"]["detection_pose"]
            object_category = locations["refrigerator"]["object_category"]
            detection_polygon = locations["refrigerator"]["detection_polygon"]
            detection_polygon = (
                detection_polygon
                if isinstance(detection_polygon, ShapelyPolygon)
                else ShapelyPolygon(
                    [(point.x, point.y) for point in detection_polygon.points]
                )
            )

            smach.StateMachine.add(
                "SAY_GOING_TO_REFRIGERATOR",
                Say(
                    text=f"I am going to the refrigerator to check for {object_category}."
                ),
                transitions={
                    "succeeded": "GO_TO_REFRIGERATOR",
                    "preempted": "GO_TO_REFRIGERATOR",
                    "aborted": "GO_TO_REFRIGERATOR",
                },
            )

            smach.StateMachine.add(
                "GO_TO_REFRIGERATOR",
                GoToLocation(location=location_pose),
                transitions={
                    "succeeded": "DETECT_OBJECTS_IN_REFRIGERATOR",
                    "failed": "GO_TO_REFRIGERATOR",
                },
            )

            smach.StateMachine.add(
                "DETECT_OBJECTS_IN_REFRIGERATOR",
                DetectAllInPolygon(
                    polygon=detection_polygon,
                    min_coverage=1.0,
                ),
                transitions={
                    "succeeded": "PROCESS_DETECTIONS_IN_REFRIGERATOR",
                    "failed": "SAY_GOING_TO_KITCHEN",
                },
                remapping={"detected_objects": "detected_objects"},
            )

            smach.StateMachine.add(
                "PROCESS_DETECTIONS_IN_REFRIGERATOR",
                self.ProcessDetectionsInLocation(
                    expected_object_category=object_category
                ),
                transitions={
                    "succeeded": "SAY_DONE_WITH_REFRIGERATOR",
                },
                remapping={
                    "detected_objects": "detected_objects",
                    "text": "text",
                },
            )

            smach.StateMachine.add(
                "SAY_DONE_WITH_REFRIGERATOR",
                Say(),
                transitions={
                    "succeeded": "SAY_GOING_TO_KITCHEN",
                    "preempted": "SAY_GOING_TO_KITCHEN",
                    "aborted": "SAY_GOING_TO_KITCHEN",
                },
                remapping={
                    "text": "text",
                },
            )

            smach.StateMachine.add(
                "SAY_GOING_TO_KITCHEN",
                Say(
                    text="I am going to the kitchen to check for people that need help."
                ),
                transitions={
                    "succeeded": "GO_TO_KITCHEN",
                    "preempted": "GO_TO_KITCHEN",
                    "aborted": "GO_TO_KITCHEN",
                },
            )

            smach.StateMachine.add(
                "GO_TO_KITCHEN",
                GoToLocation(location=kitchen_pose),
                transitions={
                    "succeeded": "SURVEY_KITCHEN",
                    "failed": "GO_TO_KITCHEN",
                },
            )

            smach.StateMachine.add(
                "SURVEY_KITCHEN",
                Survey(
                    look_range_deg=(-71.0, 71.0),
                    n_look_points=10,
                    polygon=ShapelyPolygon(
                        rospy.get_param("/gpsr/arena/rooms/kitchen/room_polygon")
                    ),
                ),
                transitions={
                    "customer_found": "SAY_PERSON_FOUND_KITCHEN",
                    "customer_not_found": "SAY_PERSON_NOT_FOUND_KITCHEN",
                },
                remapping={"hands_up_detections": "hands_up_detections"},
            )

            smach.StateMachine.add(
                "SAY_PERSON_FOUND_KITCHEN",
                Say(text="I have found a person in the kitchen that needs help."),
                transitions={
                    "succeeded": "CHOOSE_WAVING_PERSON_KITCHEN",
                    "preempted": "CHOOSE_WAVING_PERSON_KITCHEN",
                    "aborted": "CHOOSE_WAVING_PERSON_KITCHEN",
                },
                remapping={
                    "hands_up_detections": "hands_up_detections",
                },
            )

            smach.StateMachine.add(
                "SAY_PERSON_NOT_FOUND_KITCHEN",
                Say(text="I can't find anyone in the kitchen that needs help."),
                transitions={
                    "succeeded": "SAY_GOING_TO_CABINET",
                    "preempted": "SAY_GOING_TO_CABINET",
                    "aborted": "SAY_GOING_TO_CABINET",
                },
                remapping={
                    "hands_up_detections": "hands_up_detections",
                },
            )

            smach.StateMachine.add(
                "CHOOSE_WAVING_PERSON_KITCHEN",
                ChooseWavingPerson(),
                transitions={
                    "succeeded": "COMPUTE_APPROACH_POSE_KITCHEN",
                    "failed": "SAY_GOING_TO_CABINET",
                },
                remapping={
                    "hands_up_detections": "hands_up_detections",
                    "waving_person_detection": "waving_person_detection",
                },
            )

            smach.StateMachine.add(
                "COMPUTE_APPROACH_POSE_KITCHEN",
                ComputeApproach(),
                transitions={
                    "succeeded": "GO_TO_PERSON_KITCHEN",
                    "failed": "SAY_GOING_TO_CABINET",
                },
                remapping={
                    "customer_approach_pose": "customer_approach_pose",
                },
            )

            smach.StateMachine.add(
                "GO_TO_PERSON_KITCHEN",
                GoToLocation(),
                transitions={
                    "succeeded": "ASK_FOR_COMMAND_KITCHEN",
                    "failed": "GO_TO_PERSON_KITCHEN",
                },
                remapping={
                    "location": "customer_approach_pose",
                },
            )

            smach.StateMachine.add(
                "ASK_FOR_COMMAND_KITCHEN",
                CommandParserStateMachine(load_gpsr_configuration()),
                transitions={
                    "succeeded": "SAY_COMMAND_PLAN_KITCHEN",
                    "failed": "RETRY_COMMAND_KITCHEN_1",
                },
            )

            smach.StateMachine.add(
                "RETRY_COMMAND_KITCHEN_1",
                CommandParserStateMachine(load_gpsr_configuration()),
                transitions={
                    "succeeded": "SAY_COMMAND_PLAN_KITCHEN",
                    "failed": "RETRY_COMMAND_KITCHEN_2",
                },
            )

            smach.StateMachine.add(
                "RETRY_COMMAND_KITCHEN_2",
                CommandParserStateMachine(load_gpsr_configuration()),
                transitions={
                    "succeeded": "SAY_COMMAND_PLAN_KITCHEN",
                    "failed": "SAY_GOING_TO_CABINET",
                },
            )

            smach.StateMachine.add(
                "SAY_COMMAND_PLAN_KITCHEN",
                self.GetCommandString(),
                transitions={
                    "succeeded": "SAY_GOING_TO_CABINET",
                    "failed": "SAY_GOING_TO_CABINET",
                },
                remapping={
                    "parsed_command": "parsed_command",
                    "command_string": "command_string",
                },
            )


def main() -> None:
    config = load_gpsr_configuration()
    # tts_client = actionlib.SimpleActionClient("tts", TtsAction)
    # tts_client.wait_for_server()
    # _tts(tts_client, "Please open the door")
    patrol_sm = PatrolObjectLocations()
    timeout = 10.0
    start_sm = Start()
    start_sm.execute()
    rospy.sleep(3)

    outcome = patrol_sm.execute()


if __name__ == "__main__":
    rospy.init_node("gpsr_main")
    main()
    rospy.spin()
