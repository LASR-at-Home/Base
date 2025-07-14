import rospy

import smach
import smach_ros

from storing_groceries.states import (
    SelectObject,
    ScanShelves,
    AddShelvesToPlanningScene,
    ChooseShelf,
    Start,
    DetectObjects,
    GraspObject,
)

from lasr_skills import (
    Say,
    GoToLocation,
)

from lasr_manipulation_msgs.msg import PlaceGoal, PlaceAction
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion


class StoringGroceries(smach.StateMachine):

    def __init__(self, use_arm: bool = True) -> None:
        super().__init__(outcomes=["succeeded", "failed"])

        with self:

            smach.StateMachine.add(
                "START", Start(), transitions={"succeeded": "DETECT_OBJECTS"}
            )

            smach.StateMachine.add(
                "SAY_GOING_TO_TABLE",
                Say(text="I am going to the table"),
                transitions={
                    "succeeded": "GO_TO_TABLE",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "GO_TO_TABLE",
                GoToLocation(location_param="/storing_groceries/table/pose"),
                transitions={
                    "succeeded": "DETECT_OBJECTS",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "DETECT_OBJECTS",
                DetectObjects(),
                transitions={"succeeded": "SELECT_OBJECT", "failed": "failed"},
            )

            smach.StateMachine.add(
                "SELECT_OBJECT",
                SelectObject(use_arm),
                transitions={
                    "succeeded": "GRASP_OBJECT" if use_arm else "HELP_ME_GRASPING",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "GRASP_OBJECT",
                GraspObject(),
                transitions={
                    "succeeded": "HELP_ME_GRASPING",
                    "failed": "HELP_ME_GRASPING",
                },
            )

            # if not using the arm
            smach.StateMachine.add(
                "HELP_ME_GRASPING",
                Say(
                    format_str="I'm unable to grasp the {} please place it on my back. I will give you 5 seconds. 5... 4... 3... 2... 1..."
                ),
                transitions={
                    "succeeded": "GO_TO_CABINET",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"placeholders": "selected_object_name"},
            )

            smach.StateMachine.add(
                "GO_TO_CABINET",
                GoToLocation(location_param="/storing_groceries/cabinet/pose"),
                transitions={
                    "succeeded": "SCAN_SHELVES",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SCAN_SHELVES",
                ScanShelves(),
                transitions={
                    "succeeded": "CHOOSE_SHELF",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "CHOOSE_SHELF",
                ChooseShelf(use_arm),
                transitions={
                    "succeeded": (
                        "ADD_SHELVES_TO_PLANNING_SCENE"
                        if use_arm
                        else "HELP_ME_PLACING"
                    ),
                    "failed": "HELP_ME_PLACING_ANYWHERE",
                },
            )

            # if not using the arm
            smach.StateMachine.add(
                "HELP_ME_PLACING",
                Say(format_str="I can't place the {} myself."),
                transitions={
                    "succeeded": "HELP_ME_PLACING_1",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"placeholders": "selected_object_name"},
            )
            smach.StateMachine.add(
                "HELP_ME_PLACING_1",
                Say(format_str="Please place it on the {}"),
                transitions={
                    "succeeded": "SAY_GOING_TO_TABLE",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"placeholders": "chosen_shelf"},
            )

            smach.StateMachine.add(
                "HELP_ME_PLACING_ANYWHERE",
                Say(
                    format_str="I can't place the {} myself, and can't determine where to place it. Please place it on any available shelf."
                ),
                transitions={
                    "succeeded": "SAY_GOING_TO_TABLE",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"placeholders": "selected_object_name"},
            )

            smach.StateMachine.add(
                "ADD_SHELVES_TO_PLANNING_SCENE",
                AddShelvesToPlanningScene(),
                transitions={"succeeded": "PLACE_OBJECT", "failed": "failed"},
            )

            smach.StateMachine.add(
                "PLACE_OBJECT",
                smach_ros.SimpleActionState(
                    "/lasr_manipulation/place",
                    PlaceAction,
                    goal_cb=self._place_cb,
                    output_keys=["success"],
                    result_slots=["success"],
                    input_keys=["selected_object", "selected_shelf"],
                ),
                transitions={
                    "succeeded": "SAY_GOING_TO_TABLE",
                    "preempted": "failed",
                    "aborted": "failed",
                },
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
