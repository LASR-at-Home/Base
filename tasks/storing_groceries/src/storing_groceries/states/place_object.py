#!/usr/bin/env python

import rospy
import smach
import smach_ros

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3
from std_msgs.msg import Header
from std_srvs.srv import Empty as EmptySrv, EmptyRequest

from lasr_manipulation_msgs.msg import PlaceGoal, PlaceAction
from lasr_manipulation_msgs.srv import (
    AddSupportSurface,
    AddSupportSurfaceRequest,
    RemoveSupportSurface,
    RemoveSupportSurfaceRequest,
)

from lasr_skills import PlayMotion


class PrepareToPlace(smach.StateMachine):
    def __init__(self):
        super().__init__(outcomes=["succeeded"])
        with self:
            smach.StateMachine.add(
                "PLAY_PREGRASP_MOTION",
                PlayMotion(motion_name="pregrasp"),
                transitions={
                    "succeeded": "succeeded",
                    "preempted": "succeeded",
                    "aborted": "succeeded",
                },
            )


class PreparePlanningScene(smach.StateMachine):
    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"], input_keys=["selected_shelf"])

        with self:
            smach.StateMachine.add(
                "CLEAR_PLANNING_SCENE",
                smach_ros.ServiceState(
                    "/lasr_manipulation_planning_scene/clear",
                    EmptySrv,
                    request=EmptyRequest(),
                ),
                transitions={
                    "succeeded": "ADD_SHELF_SURFACE",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "ADD_SHELF_SURFACE",
                smach_ros.ServiceState(
                    "/lasr_manipulation_planning_scene/add_support_surface",
                    AddSupportSurface,
                    request_cb=self._build_add_surface_request,
                    output_keys=["success"],
                    response_slots=["success"],
                ),
                transitions={
                    "succeeded": "succeeded",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

    def _build_add_surface_request(self, userdata, request):
        return AddSupportSurfaceRequest(
            surface_name="shelf",
            pose=PoseStamped(
                header=Header(frame_id="map"),
                pose=Pose(
                    position=Point(**rospy.get_param("/storing_groceries/shelf/surface/pose/position")),
                    orientation=Quaternion(**rospy.get_param("/storing_groceries/shelf/surface/pose/orientation")),
                ),
            ),
            dimensions=Vector3(**rospy.get_param("/storing_groceries/shelf/surface/dimensions"))
        )


class PlaceObject(smach.StateMachine):
    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["selected_object", "selected_shelf"]
        )

        with self:
            prepare_place = smach.Concurrence(
                outcomes=["succeeded", "failed"],
                default_outcome="failed",
                input_keys=["selected_object", "selected_shelf"],
                outcome_map={
                    "succeeded": {
                        "PREPARE_TO_PLACE": "succeeded",
                        "PREPARE_PLANNING_SCENE": "succeeded",
                    },
                    "failed": {
                        "PREPARE_PLANNING_SCENE": "failed",
                    },
                },
            )

            with prepare_place:
                smach.Concurrence.add("PREPARE_TO_PLACE", PrepareToPlace())
                smach.Concurrence.add("PREPARE_PLANNING_SCENE", PreparePlanningScene())

            smach.StateMachine.add(
                "PREPARE_PLACE",
                prepare_place,
                transitions={"succeeded": "PLACE_OBJECT", "failed": "failed"},
            )

            smach.StateMachine.add(
                "PLACE_OBJECT",
                smach_ros.SimpleActionState(
                    "/lasr_manipulation/place",
                    PlaceAction,
                    goal_cb=self._place_cb,
                    input_keys=["selected_object", "selected_shelf"],
                ),
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
                    "succeeded": "REMOVE_SHELF_FROM_PLANNING_SCENE",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "REMOVE_SHELF_FROM_PLANNING_SCENE",
                smach_ros.ServiceState(
                    "/lasr_manipulation_planning_scene/remove_support_surface",
                    RemoveSupportSurface,
                    request_cb=self._remove_shelf_support_surface_cb,
                    output_keys=["success"],
                    response_slots=["success"],
                ),
                transitions={
                    "succeeded": "succeeded",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

    def _place_cb(self, userdata, goal):
        shelf_ns = userdata.selected_shelf
        param_prefix = f"/cabinet/shelves/{shelf_ns}"
        data = rospy.get_param(param_prefix)

        goal = PlaceGoal()
        goal.object_id = userdata.selected_object[0].name
        goal.surface_id = data["surface_id"]

        for pose_data in data["candidate_poses"]:
            ps = PoseStamped()
            ps.header.frame_id = pose_data["header"]["frame_id"]
            ps.pose.position.x = pose_data["pose"]["position"]["x"]
            ps.pose.position.y = pose_data["pose"]["position"]["y"]
            ps.pose.position.z = pose_data["pose"]["position"]["z"]
            ps.pose.orientation.x = pose_data["pose"]["orientation"]["x"]
            ps.pose.orientation.y = pose_data["pose"]["orientation"]["y"]
            ps.pose.orientation.z = pose_data["pose"]["orientation"]["z"]
            ps.pose.orientation.w = pose_data["pose"]["orientation"]["w"]
            goal.candidate_poses.append(ps)

        return goal

    def _remove_shelf_support_surface_cb(self, userdata, request):
        req = RemoveSupportSurfaceRequest()
        req.surface_name = "shelf"
        return req


if __name__ == "__main__":
    rospy.init_node("place_object_state_machine")

    # Example usage:
    sm = PlaceObject()
    sm.userdata.selected_object = [type("Obj", (), {"name": "object_1"})()]  
    sm.userdata.selected_shelf = "second_top"

    outcome = sm.execute()
