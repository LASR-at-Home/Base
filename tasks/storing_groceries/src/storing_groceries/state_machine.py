import rospy

import smach
import smach_ros
from shapely import Polygon as ShapelyPolygon

from storing_groceries.states import SetupPlanningScene, SelectObject, SegmentObject

from lasr_skills import (
    Say,
    DetectDoorOpening,
    GoToLocation,
    DetectAllInPolygonSensorData,
    PlayMotion,
)

from lasr_manipulation_msgs.srv import (
    Registration,
    RegistrationRequest,
    AddCollisionObject,
    AddCollisionObjectRequest,
)

from lasr_manipulation_msgs.msg import PickAction, PickGoal

from std_msgs.msg import Empty


class StoringGroceries(smach.StateMachine):

    def __init__(self) -> None:
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
                    "succeeded": "SETUP_PLANNING_SCENE",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "SETUP_PLANNING_SCENE",
                SetupPlanningScene(),
                transitions={
                    "succeeded": "SAY_WAITING",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_WAITING",
                Say(text="Waiting for the door to open."),
                transitions={
                    "succeeded": "WAIT_FOR_DOOR_TO_OPEN",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "WAIT_FOR_DOOR_TO_OPEN",
                DetectDoorOpening(),
                transitions={
                    "door_opened": "SAY_GOING_TO_TABLE",
                },
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
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "DETECT_OBJECTS",
                DetectAllInPolygonSensorData(
                    ShapelyPolygon(rospy.get_param("/storing_groceries/table/polygon")),
                    object_filter=[
                        k for k in rospy.get_param("/storing_groceries/objects")
                    ],
                    min_coverage=1.0,
                    min_confidence=0.7,
                ),
            )

            smach.StateMachine.add(
                "SELECT_OBJECT",
                SelectObject(),
                transitions={"succeeded": "SEGMENT_OBJECT", "failed": "failed"},
            )

            smach.StateMachine.add(
                "SEGMENT_OBJECT",
                SegmentObject(),
                transitions={"succeeded": "REGISTER_OBJECT", "failed": "failed"},
            )

            smach.StateMachine.add(
                "REGISTER_OBJECT",
                smach_ros.ServiceState(
                    "/lasr_manipulation/registration",
                    Registration,
                    request_cb=self._register_object_cb,
                    output_keys=["transform", "scale", "success"],
                    response_slots=["transform", "scale", "success"],
                    input_keys=["masked_cloud", "selected_object"],
                ),
                transitions={
                    "succeeded": "ADD_COLLISION_OBJECT",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "ADD_COLLISION_OBJECT",
                smach_ros.ServiceState(
                    "/lasr_manipulation_planning_scene/add_collision_object",
                    AddCollisionObject,
                    request_cb=self._add_collision_object_cb,
                    output_keys=["success"],
                    response_slots=["success"],
                    input_keys=["selected_object", "transform", "scale"],
                ),
                transitions={
                    "succeeded": "PLAY_PREGRASP_MOTION",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "PLAY_PREGRASP_MOTION",
                PlayMotion(motion_name="pregrasp"),
                transitions={
                    "succeeded": "PICK_OBJECT",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "PICK_OBJECT",
                smach_ros.SimpleActionState(
                    "/lasr_manipulation/pick",
                    PickAction,
                    goal_cb=self._pick_cb,
                    output_keys=["success", "grasp_pose"],
                    result_slots=["success", "grasp_pose"],
                    input_keys=["selected_object", "transform", "scale"],
                ),
                transitions={
                    "succeeded": "PLAY_PREGRASP_MOTION_POSTGRASP",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "PLAY_PREGRASP_MOTION_POSTGRASP",
                PlayMotion(motion_name="pregrasp"),
                transitions={
                    "succeeded": "succeeded",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

    def _register_object_cb(self, userdata):
        return RegistrationRequest(
            userdata.masked_cloud, userdata.selected_object[0].name, 10
        )

    def _add_collision_object_cb(self, userdata):
        return AddCollisionObjectRequest(
            userdata.selected_object[0].name,
            userdata.selected_object[0].name,
            userdata.transform,
            userdata.scale,
        )

    def _pick_cb(self, userdata):
        return PickGoal(
            userdata.selected_object[0].name,
            userdata.selected_object[0].name,
            userdata.transform,
            userdata.scale,
        )
