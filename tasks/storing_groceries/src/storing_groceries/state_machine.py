import rospy

import smach
import smach_ros
from shapely import Polygon as ShapelyPolygon

from storing_groceries.states import (
    SelectObject,
    SegmentObject,
    ScanShelves,
    AddShelvesToPlanningScene,
    ChooseShelf,
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
    DetectAndAddSupportSurfaceRequest,
    RemoveSupportSurface,
    RemoveSupportSurfaceRequest,
    DetachObjectFromGripperRequest,
)

from lasr_manipulation_msgs.msg import PickAction, PickGoal, PlaceGoal, PlaceAction
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Empty, Header
from std_srvs.srv import Empty as EmptySrv, EmptyRequest
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3
from lasr_manipulation_msgs.srv import AddSupportSurface, AddSupportSurfaceRequest


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
                    "aborted": "failed",
                    "preempted": "failed",
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
                    "succeeded": (
                        "CLEAR_PLANNING_SCENE_AT_TABLE" if use_arm else "LOOK_AT_TABLE"
                    ),
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "CLEAR_PLANNING_SCENE_AT_TABLE",
                smach_ros.ServiceState(
                    "/lasr_manipulation_planning_scene/clear",
                    EmptySrv,
                    request=EmptyRequest(),
                ),
                transitions={
                    "succeeded": "ADD_TABLE_SURFACE",
                    "preempted": "failed",
                    "aborted": "failed",
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
                transitions={
                    "succeeded": "SEGMENT_OBJECT" if use_arm else "HELP_ME_GRASPING",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SEGMENT_OBJECT",
                SegmentObject(),
                transitions={
                    "succeeded": "REGISTER_OBJECT",
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
                    format_str="I'm unable to grasp the {} please place it on my back. I will give you 5 seconds. 5... 4... 3... 2... 1..."
                ),
                transitions={
                    "succeeded": "ASK_OPEN_CABINET_DOOR",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"placeholders": "selected_object_name"},
            )

            smach.StateMachine.add(
                "ASK_OPEN_CABINET_DOOR",
                Say(text="Please open both cabinet doors fully."),
                transitions={
                    "succeeded": "WAIT_FOR_CABINET_DOOR_OPEN",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "WAIT_FOR_CABINET_DOOR_OPEN",
                smach.CBState(lambda ud: rospy.sleep(5.0) or "succeeded"),
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
                    "succeeded": "GO_TO_TABLE",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"placeholders": "selected_shelf"},
            )

            smach.StateMachine.add(
                "HELP_ME_PLACING_ANYWHERE",
                Say(
                    format_str="I can't place the {} myself, and can't determine where to place it. Please place it on any available shelf."
                ),
                transitions={
                    "succeeded": "GO_TO_TABLE",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"placeholders": "selected_object_name"},
            )

            smach.StateMachine(
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

    def _pick_cb(self, userdata, goal):
        return PickGoal(
            userdata.selected_object[0].name,
            userdata.selected_object[0].name,
            userdata.transform,
            userdata.scale,
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

    def _remove_table_support_surface_cb(self, userdata, request):
        return RemoveSupportSurfaceRequest("table")
