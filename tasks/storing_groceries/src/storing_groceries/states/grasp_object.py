import rospy
import smach
import smach_ros
from shapely import Polygon as ShapelyPolygon

from lasr_skills import PlayMotion
from storing_groceries.states import SegmentObject

from lasr_manipulation_msgs.srv import (
    Registration,
    RegistrationRequest,
    AddCollisionObject,
    AddCollisionObjectRequest,
    AddSupportSurface,
    AddSupportSurfaceRequest,
)

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3
from std_msgs.msg import Header
from std_srvs.srv import Empty as EmptySrv, EmptyRequest
from lasr_manipulation_msgs.msg import PickAction, PickGoal, PlaceGoal, PlaceAction
from lasr_manipulation_msgs.srv import (
    RemoveSupportSurface,
    RemoveSupportSurfaceRequest,
)


class PrepareToGrasp(smach.StateMachine):

    def __init__(self):

        super().__init__(outcomes=["succeeded"])

        with self:

            smach.StateMachine.add(
                "PLAY_PREGRASP_MOTION",
                PlayMotion(motion_name="pregrasp"),
                transitions={
                    "succeeded": "OPEN_GRIPPER",
                    "preempted": "OPEN_GRIPPER",
                    "aborted": "OPEN_GRIPPER",
                },
            )

            smach.StateMachine.add(
                "OPEN_GRIPPER",
                PlayMotion(motion_name="open_gripper"),
                transitions={
                    "succeeded": "succeeded",
                    "preempted": "succeeded",
                    "aborted": "succeeded",
                },
            )


class PreparePlanningScene(smach.StateMachine):

    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "failed"], input_keys=["selected_object"]
        )

        with self:

            smach.StateMachine.add(
                "CLEAR_PLANNING_SCENE",
                smach_ros.ServiceState(
                    "/lasr_manipulation_planning_scene/clear",
                    EmptySrv,
                    request=EmptyRequest(),
                ),
                transitions={
                    "succeeded": "ADD_TABLE_SURFACE",
                    "preempted": "failed",  # TODO: handle failure properly
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
                    "succeeded": "SEGMENT_OBJECT",
                    "preempted": "failed",  # TODO: handle failure properly
                    "aborted": "failed",
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
                    "preempted": "failed",  # TODO: handle failure properly
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
                    "succeeded": "succeeded",
                    "preempted": "failed",  # TODO: handle failure properly
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


class GraspObject(smach.StateMachine):

    def __init__(self):

        super().__init__(
            outcomes=["succeeded", "failed"], input_keys=["selected_object"]
        )

        with self:

            prepare_grasp_sm = smach.Concurrence(outcomes=["succeeded", "failed"])
            with prepare_grasp_sm:
                smach.Concurrence.add("PREPARE_TO_GRASP", PrepareToGrasp())
                smach.Concurrence.add("PREPARE_PLANNING_SCENE", PreparePlanningScene())

            smach.StateMachine.add(
                "PREPARE_GRASP",
                prepare_grasp_sm,
                transitions={"succeeded": "PICK_OBJECT", "failed": "failed"},
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
                    "succeeded": "succeeded",
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

    def _pick_cb(self, userdata, goal):
        return PickGoal(
            userdata.selected_object[0].name,
            userdata.selected_object[0].name,
            userdata.transform,
            userdata.scale,
        )

    def _remove_table_support_surface_cb(self, userdata, request):
        return RemoveSupportSurfaceRequest("table")
