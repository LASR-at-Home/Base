#!/usr/bin/env python
import rospy
import smach
import smach_ros

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header  
from lasr_skills import ReceiveObject, GoToLocation, PlayMotion
from lasr_manipulation_msgs.msg import PlaceAction, PlaceGoal


class HandoverAndDeliver(smach.StateMachine):
    def __init__(self):
        super(HandoverAndDeliver, self).__init__(
            outcomes=["succeeded", "failed", "escape"],
            input_keys=[
                "selected_object",
                "place_pose",
                "surface_id",
                "goal_pose",
                "object_name", 
            ],
        )

        with self:
            smach.StateMachine.add(
                "RECEIVE_OBJECT",
                ReceiveObject(),
                transitions={
                    "succeeded": "DELIVER_OBJECT",
                    "failed": "DELIVER_OBJECT",
                },
                remapping={
                    "object_name": "object_name",
                    "placeholders": "placeholders",
                },
            )

            smach.StateMachine.add(
                "DELIVER_OBJECT",
                GoToLocation(),
                transitions={
                    "succeeded": "PLAY_PREPLACE_MOTION",
                    "failed": "PLAY_PREPLACE_MOTION",
                },
                remapping={"goal_pose": "goal_pose"},
            )

            smach.StateMachine.add(
                "PLAY_PREPLACE_MOTION",
                PlayMotion(motion_name="home_to_preplace"),
                transitions={
                    "succeeded": "PLACE_OBJECT",
                    "preempted": "PLACE_OBJECT",
                    "aborted": "PLACE_OBJECT",
                },
            )

            smach.StateMachine.add(
                "PLACE_OBJECT",
                smach_ros.SimpleActionState(
                    "/lasr_manipulation/place",
                    PlaceAction,
                    goal_cb=self._place_cb,
                    input_keys=["selected_object", "place_pose", "surface_id"],
                ),
                transitions={
                    "succeeded": "OPEN_GRIPPER",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )


            smach.StateMachine.add(
                "OPEN_GRIPPER",
                PlayMotion(motion_name="open_gripper"),
                transitions={
                    "succeeded": "HOME_POSITION",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "HOME_POSITION",
                PlayMotion(motion_name="home"),
                transitions={
                    "succeeded": "succeeded",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )


    def _place_cb(self, userdata, goal):
        goal = PlaceGoal()
        goal.object_id = (
            userdata.selected_object.name
            if hasattr(userdata.selected_object, "name")
            else userdata.selected_object
        )
        goal.pose = userdata.place_pose
        goal.surface_id = userdata.surface_id
        return goal


if __name__ == "__main__":
    rospy.init_node("handover_and_deliver_test")

    sm = HandoverAndDeliver()

    # Fake object with a .name attribute
    sm.userdata.selected_object = type("Obj", (), {"name": "object_1"})()
    sm.userdata.object_name = sm.userdata.selected_object.name

    # Proper PoseStamped with Header
    sm.userdata.place_pose = PoseStamped(
        header=Header(frame_id="base_footprint"),
        pose=Pose(
            position=Point(x=0.7, y=-0.15, z=0.95),
            orientation=Quaternion(w=1.0),
        ),
    )

    sm.userdata.goal_pose = PoseStamped(
        header=Header(frame_id="map"),
        pose=Pose(
            position=Point(x=1.0, y=0.5, z=0.0),
            orientation=Quaternion(w=1.0),
        ),
    )

    sm.userdata.surface_id = "shelf"
    sm.userdata.placeholders = {"object_name": sm.userdata.object_name}

    outcome = sm.execute()
    rospy.loginfo(f"State machine finished with outcome: {outcome}")
