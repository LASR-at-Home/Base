#!/usr/bin/env python
import rospy

import smach
import smach_ros
from shapely import Polygon as ShapelyPolygon

from storing_groceries.states import (
    SelectObject,
    ScanShelves,
    AddShelvesToPlanningScene,
    ChooseShelf,
    Start,
    DetectObjects,
    GraspObject,
    FindAndGoToTable,
    SelectAndVisualiseObject,
)

from lasr_skills import (
    Say,
    GoToLocation,
    DetectDoorOpening,
    DetectAllInPolygonSensorData,
)

from lasr_manipulation_msgs.msg import PlaceGoal, PlaceAction
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Empty


class StoringGroceries(smach.StateMachine):

    def __init__(self, use_arm: bool = True) -> None:
        super().__init__(outcomes=["succeeded", "failed"])
        with self:

            smach.StateMachine.add(
                "WAIT_START",
                smach_ros.MonitorState(
                    "/storing_groceries/start",
                    Empty,
                    lambda *_: False,
                ),
                transitions={
                    "valid": "WAIT_START",
                    "preempted": "WAIT_START",
                    "invalid": "SAY_START",
                },
            )

            smach.StateMachine.add(
                "SAY_START",
                Say(text="Start of Storing Groceries task."),
                transitions={
                    "succeeded": "SAY_WAITING",
                    "aborted": "SAY_WAITING",
                    "preempted": "SAY_WAITING",
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
                    "door_opened": "SAY_GOING_TO_CABINET",
                },
            )

            smach.StateMachine.add(
                "SAY_GOING_TO_CABINET",
                Say(text="I am going to the cabinet."),
                transitions={
                    "succeeded": "GO_TO_CABINET",
                    "aborted": "GO_TO_CABINET",
                    "preempted": "GO_TO_CABINET",
                },
            )

            smach.StateMachine.add(
                "GO_TO_CABINET",
                GoToLocation(location_param="/storing_groceries/cabinet/pose"),
                transitions={
                    "succeeded": "SAY_OPEN_CABINETS",
                    "failed": "SAY_OPEN_CABINETS",
                },
            )

            smach.StateMachine.add(
                "SAY_OPEN_CABINETS",
                Say(
                    text="Referee, I am unable to open the cabinets. Please open both cabinets for me. I will give you 10 seconds. 10.. 9.. 8.. 7.. 6.. 5.. 4.. 3.. 2.. 1.. "
                ),
                transitions={
                    "succeeded": "SCAN_SHELVES",
                    "preempted": "SCAN_SHELVES",
                    "aborted": "SCAN_SHELVES",
                },
            )

            smach.StateMachine.add(
                "SCAN_SHELVES",
                ScanShelves(),
                transitions={
                    "succeeded": "FIND_TABLE",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "FIND_TABLE",
                FindAndGoToTable(),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )

            smach.StateMachine.add(
                "DETECT_OBJECTS",
                DetectAllInPolygonSensorData(
                    ShapelyPolygon(
                        rospy.get_param("/storing_groceries/table/search_polygon")
                    ),
                    object_filter=[
                        k for k in rospy.get_param("/storing_groceries/objects")
                    ],
                    min_coverage=1.0,
                    min_confidence=0.1,
                    z_sweep_min=0.0,
                    z_sweep_max=1.0,
                    model="robocup.pt",
                ),
                transitions={"succeeded": "succeeded", "failed": "DETECT_OBJECTS"},
            )

            smach.StateMachine.add(
                "SELECT_OBJECT",
                SelectAndVisualiseObject(),
                transitions={
                    "succeeded": "HELP_ME_GRASPING",
                    "failed": "DETECT_OBJECTS",
                },
            )

            smach.StateMachine.add(
                "HELP_ME_GRASPING",
                Say(
                    format_str="I'm unable to grasp the {} please place it on my back. I will give you 5 seconds. 5... 4... 3... 2... 1..."
                ),
                transitions={
                    "succeeded": "GO_TO_CABINET_1",
                    "aborted": "GO_TO_CABINET_1",
                    "preempted": "GO_TO_CABINET_1",
                },
                remapping={"placeholders": "selected_object_name"},
            )

            smach.StateMachine.add(
                "GO_TO_CABINET_1",
                GoToLocation(location_param="/storing_groceries/cabinet/pose"),
                transitions={
                    "succeeded": "CHOOSE_SHELF",
                    "failed": "GO_TO_CABINET_1",
                },
            )

            smach.StateMachine.add(
                "CHOOSE_SHELF",
                ChooseShelf(use_arm),
                transitions={
                    "succeeded": "HELP_ME_PLACING",
                    "failed": "HELP_ME_PLACING_ANYWHERE",
                },
            )

            smach.StateMachine.add(
                "HELP_ME_PLACING",
                Say(format_str="I can't place the {} myself."),
                transitions={
                    "succeeded": "HELP_ME_PLACING_1",
                    "aborted": "HELP_ME_PLACING_1",
                    "preempted": "HELP_ME_PLACING_1",
                },
                remapping={"placeholders": "selected_object_name"},
            )

            smach.StateMachine.add(
                "HELP_ME_PLACING_1",
                Say(format_str="Please place it on the {} shelf."),
                transitions={
                    "succeeded": "SAY_GOING_TO_TABLE_1",
                    "aborted": "SAY_GOING_TO_TABLE_1",
                    "preempted": "SAY_GOING_TO_TABLE_1",
                },
                remapping={"placeholders": "chosen_shelf"},
            )

            smach.StateMachine.add(
                "HELP_ME_PLACING_ANYWHERE",
                Say(
                    format_str="I can't place the {} myself, and can't determine where to place it. Please place it on any available shelf."
                ),
                transitions={
                    "succeeded": "SAY_GOING_TO_TABLE_1",
                    "aborted": "SAY_GOING_TO_TABLE_1",
                    "preempted": "SAY_GOING_TO_TABLE_1",
                },
                remapping={"placeholders": "selected_object_name"},
            )

            smach.StateMachine.add(
                "SAY_GOING_TO_TABLE_1",
                Say(text="I am going to the table"),
                transitions={
                    "succeeded": "GO_TO_TABLE_1",
                    "aborted": "GO_TO_TABLE_1",
                    "preempted": "GO_TO_TABLE_1",
                },
            )

            smach.StateMachine.add(
                "GO_TO_TABLE_1",
                GoToLocation(),
                remapping={"location": "table_pose"},
                transitions={"succeeded": "DETECT_OBJECTS", "failed": "GO_TO_TABLE_1"},
            )

    def _place_cb(self, userdata, goal):
        place_poses = [
            PoseStamped(
                pose=Pose(
                    position=Point(**p["position"]),
                    orientation=Quaternion(**p["orientation"]),
                ),
                header=Header(frame_id="map"),
            )
            for p in rospy.get_param(
                f"/storing_groceries/cabinet/shelves/{userdata.selected_shelf}/place_candidates"
            )
        ]
        return PlaceGoal(
            userdata.selected_object[0].name, userdata.selected_shelf, place_poses
        )
