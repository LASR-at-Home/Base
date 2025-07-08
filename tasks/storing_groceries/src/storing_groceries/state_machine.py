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
    LookToPoint,
)

from lasr_manipulation_msgs.srv import (
    Registration,
    RegistrationRequest,
    AddCollisionObject,
    AddCollisionObjectRequest,
    DetectAndAddSupportSurface,
    DetectAndAddSupportSurfaceRequest,
    RemoveSupportSurface,
    RemoveSupportSurfaceRequest,
    DetachObjectFromGripperRequest,
    DetachObjectFromGripper,
)

from lasr_manipulation_msgs.msg import PickAction, PickGoal
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Empty, Header


"""
Looks like we can't have everything setup in the planning scene apriori, we need to dynamically add things, as they are all in the base footprint...
"""


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
                DetectDoorOpening(timeout=1.0),
                transitions={
                    "door_opened": "SAY_GOING_TO_TABLE",
                },
            )

            smach.StateMachine.add(
                "SAY_GOING_TO_TABLE",
                Say(text="I am going to the table"),
                transitions={
                    "succeeded": "LOOK_AT_TABLE",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "GO_TO_TABLE",
                GoToLocation(location_param="/storing_groceries/table/pose"),
                transitions={
                    "succeeded": "LOOK_AT_TABLE",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "LOOK_AT_TABLE",
                LookToPoint(
                    pointstamped=PointStamped(
                        point=Point(
                            **rospy.get_param("/storing_groceries/table/pose/position")
                        ),
                        header=Header(frame_id="map"),
                    )
                ),
                transitions={
                    "succeeded": "DETECT_OBJECTS",
                    "aborted": "failed",
                    "timed_out": "failed",
                },
            )

            smach.StateMachine.add(
                "DETECT_OBJECTS",
                DetectAllInPolygonSensorData(
                    ShapelyPolygon(rospy.get_param("/storing_groceries/table/polygon")),
                    object_filter=[
                        k for k in rospy.get_param("/storing_groceries/objects")
                    ],
                    min_coverage=0.9,
                    min_confidence=0.1,
                    z_axis=0.8,
                    model="lasr.pt",
                ),
                transitions={"succeeded": "SELECT_OBJECT", "failed": "failed"},
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
                    "succeeded": "ADD_TABLE_SUPPORT_SURFACE",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "ADD_TABLE_SUPPORT_SURFACE",
                smach_ros.ServiceState(
                    "/lasr_manipulation_planning_scene/detect_and_add_support_surface",
                    DetectAndAddSupportSurface,
                    request_cb=self._detect_and_add_table_support_surface_cb,
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
                    "succeeded": "OPEN_GRIPPER",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "OPEN_GRIPPER",
                PlayMotion(motion_name="open_gripper"),
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
                    "succeeded": "PLAY_HOME_MOTION",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "PLAY_HOME_MOTION",
                PlayMotion(motion_name="home"),
                transitions={
                    "succeeded": "GO_TO_CABINET",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            # smach.StateMachine.add(
            #     "REMOVE_TABLE_SUPPORT_SURFACE",
            #     smach_ros.ServiceState(
            #         "/lasr_manipulation_planning_scene/remove_support_surface",
            #         RemoveSupportSurface,
            #         request_cb=self._remove_table_support_surface_cb,
            #         output_keys=["success"],
            #         response_slots=["success"],
            #     ),
            #     transitions={
            #         "succeeded": "GO_TO_CABINET",
            #         "preempted": "failed",
            #         "aborted": "failed",
            #     },
            # )

            smach.StateMachine.add(
                "GO_TO_CABINET",
                GoToLocation(location_param="/storing_groceries/cabinet/pose"),
                transitions={
                    "succeeded": "SAY_PLACE",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_PLACE",
                Say(
                    "Please grab the object from my gripper and place it on the shelf. I will give you 5 seconds. 5... 4... 3... 2... 1..."
                ),
                transitions={
                    "succeeded": "OPEN_GRIPPER_RELEASE_OBJECT",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "OPEN_GRIPPER_RELEASE_OBJECT",
                PlayMotion(motion_name="open_gripper"),
                transitions={
                    "succeeded": "DETACH_OBJECT_FROM_GRIPPER",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "DETACH_OBJECT_FROM_GRIPPER",
                smach_ros.ServiceState(
                    "/lasr_manipulation_planning_scene/detach_object_from_gripper",
                    DetachObjectFromGripper,
                    request_cb=self._detach_object_from_gripper_cb,
                    output_keys=[],
                    input_keys=["selected_object"],
                ),
                transitions={
                    "succeeded": "GO_TO_TABLE",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

    def _register_object_cb(self, userdata, request):
        return RegistrationRequest(
            userdata.masked_cloud,
            userdata.selected_object[0].name,
            rospy.get_param(
                f"/storing_groceries/objects/{userdata.selected_object[0].name}/scale"
            ),
            25,
        )

    def _add_collision_object_cb(self, userdata, request):
        return AddCollisionObjectRequest(
            userdata.selected_object[0].name,
            userdata.selected_object[0].name,
            userdata.transform,
            userdata.scale,
        )

    def _detect_and_add_table_support_surface_cb(self, userdata, request):
        return DetectAndAddSupportSurfaceRequest(
            "table",
            userdata.selected_object[1],
            userdata.selected_object[0].name,
            userdata.transform,
            userdata.scale,
        )

    def _pick_cb(self, userdata, goal):
        return PickGoal(
            userdata.selected_object[0].name,
            userdata.selected_object[0].name,
            userdata.transform,
            userdata.scale,
        )

    def _remove_table_support_surface_cb(self, userdata, request):
        return RemoveSupportSurfaceRequest("table")

    def _detach_object_from_gripper_cb(self, userdata, request):
        return DetachObjectFromGripperRequest(userdata.selected_object[0].name)
