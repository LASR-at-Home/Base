import rospy

import smach
import smach_ros
from shapely import Polygon as ShapelyPolygon

from storing_groceries.states import (
    SelectObject,
    SegmentObject,
    ScanShelves,
    AddShelvesToPlanningScene,
)

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

from lasr_manipulation_msgs.msg import PickAction, PickGoal, PlaceGoal, PlaceAction
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Empty, Header
from std_srvs.srv import Empty as EmptySrv, EmptyRequest
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3
from lasr_manipulation_msgs.srv import AddSupportSurface, AddSupportSurfaceRequest


"""
Looks like we can't have everything setup in the planning scene apriori, we need to dynamically add things, as they are all in the base footprint...
We need to lift the object off the surface first. I can't understand why, but sometimes the planner fucks up. I think this is because the support surface is touching/colliding with  the object.
"""


class StoringGroceries(smach.StateMachine):

    def __init__(self, use_arm=True) -> None:
        super().__init__(outcomes=["succeeded", "failed"])
        self.use_arm = use_arm

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
                    "succeeded": (
                        "CLEAR_PLANNING_SCENE" if self.use_arm else "SAY_WAITING"
                    ),
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "CLEAR_PLANNING_SCENE",
                smach_ros.ServiceState(
                    "/lasr_manipulation_planning_scene/clear",
                    EmptySrv,
                    request=EmptyRequest(),
                ),
                transitions={
                    "succeeded": "SAY_WAITING",
                    "preempted": "failed",
                    "aborted": "failed",
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
                    "succeeded": "GO_TO_TABLE",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "GO_TO_TABLE",
                GoToLocation(location_param="/storing_groceries/table/pose"),
                transitions={
                    "succeeded": "ADD_TABLE_SURFACE",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "ADD_TABLE_SURFACE",
                smach_ros.ServiceState(
                    "/lasr_manipulation_planning_scene/add_support_surface",
                    AddSupportSurface,
                    request=AddSupportSurfaceRequest(
                        "table",
                        PoseStamped(
                            header=Header(frame_id="map"),
                            pose=Pose(
                                position=Point(
                                    **rospy.get_param(
                                        "/storing_groceries/table/surface/pose/position"
                                    ),
                                ),
                                orientation=Quaternion(
                                    **rospy.get_param(
                                        "/storing_groceries/table/surface/pose/orientation"
                                    )
                                ),
                            ),
                        ),
                        Vector3(
                            **rospy.get_param(
                                "/storing_groceries/table/surface/dimensions"
                            )
                        ),
                    ),
                    output_keys=["success"],
                    response_slots=["success"],
                ),
                transitions={
                    "succeeded": "LOOK_AT_TABLE",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "LOOK_AT_TABLE",
                LookToPoint(
                    pointstamped=PointStamped(
                        point=Point(
                            **rospy.get_param("/storing_groceries/table/look_point")
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
                transitions={
                    "succeeded": (
                        "REGISTER_OBJECT" if self.use_arm else "HELP_ME_GRASPING"
                    ),
                    "failed": "failed",
                },
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
                    "succeeded": "ADD_TABLE_TO_PLANNING_SCENE",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "ADD_TABLE_TO_PLANNING_SCENE",
                smach_ros.ServiceState(
                    "/lasr_manipulation_planning_scene/add_support_surface",
                    AddSupportSurface,
                    request=AddSupportSurfaceRequest(
                        "table",
                        PoseStamped(
                            header=Header(frame_id="map"),
                            pose=Pose(
                                position=Point(
                                    **rospy.get_param(
                                        "/storing_groceries/table/surface/pose/position"
                                    ),
                                ),
                                orientation=Quaternion(
                                    **rospy.get_param(
                                        "/storing_groceries/table/surface/pose/orientation"
                                    )
                                ),
                            ),
                        ),
                        Vector3(
                            **rospy.get_param(
                                "/storing_groceries/table/surface/dimensions"
                            )
                        ),
                    ),
                    output_keys=["success"],
                    response_slots=["success"],
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
                    "succeeded": "REMOVE_TABLE_FROM_PLANNING_SCENE",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "REMOVE_TABLE_FROM_PLANNING_SCENE",
                smach_ros.ServiceState(
                    "/lasr_manipulation_planning_scene/remove_support_surface",
                    RemoveSupportSurface,
                    request_cb=self._remove_table_support_surface_cb,
                    output_keys=["success"],
                    response_slots=["success"],
                ),
                transitions={
                    "succeeded": "ASK_OPEN_CABINET_DOOR",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            # if not using the arm
            smach.StateMachine.add(
                "HELP_ME_GRASPING",
                Say(
                    text="I'm not able to grasping the objects myself, would you please place the groceries on my back?"
                ),
                transitions={
                    "succeeded": "ASK_OPEN_CABINET_DOOR",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "ASK_OPEN_CABINET_DOOR",
                Say(text="Could you please open the cabinet door for me?"),
                transitions={
                    "succeeded": "WAIT_FOR_CABINET_DOOR_OPEN",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "WAIT_FOR_CABINET_DOOR_OPEN",
                smach.CBState(lambda ud: rospy.sleep(3.0) or "succeeded"),
                transitions={"succeeded": "GO_TO_CABINET"},
            )

            smach.StateMachine.add(
                "GO_TO_CABINET",
                GoToLocation(location_param="/storing_groceries/cabinet/pose"),
                transitions={
                    "succeeded": "SCAN_SHELVES",
                    "failed": "failed",
                },
            )

            smach.StateMachine(
                "ADD_SHELVES_TO_PLANNING_SCENE",
                AddShelvesToPlanningScene(),
                transitions={"succeeded": "SCAN_SHELVES", "failed": "failed"},
            )

            smach.StateMachine.add(
                "SCAN_SHELVES",
                ScanShelves(),
                transitions={
                    "succeeded": "PLACE_OBJECT" if self.use_arm else "HELP_ME_PLACING",
                    "failed": "failed",
                },
            )

            # if not using the arm
            smach.StateMachine.add(
                "HELP_ME_PLACING",
                Say(
                    text="I'm not able to placing the objects myself, would you please place the groceries on the shelf for me?"
                ),
                transitions={
                    "succeeded": "GO_TO_TABLE",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "PLACE_OBJECT",
                smach_ros.SimpleActionState(
                    "/lasr_manipulation/place",
                    PlaceAction,
                    goal_cb=self._place_cb,
                    output_keys=["success"],
                    result_slots=["success"],
                    input_keys=["selected_object", "surface", "candidate_poses"],
                ),
                transitions={
                    "succeeded": "OPEN_GRIPPER_RELEASE_OBJECT",
                    "preempted": "failed",
                    "aborted": "failed",
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

    def _place_cb(self, userdata, goal):
        return PlaceGoal(
            userdata.selected_object[0].name,
            userdata.surfaceid,
            userdata.candidate_poses,
        )

    def _remove_table_support_surface_cb(self, userdata, request):
        return RemoveSupportSurfaceRequest("table")

    def _detach_object_from_gripper_cb(self, userdata, request):
        return DetachObjectFromGripperRequest(userdata.selected_object[0].name)
