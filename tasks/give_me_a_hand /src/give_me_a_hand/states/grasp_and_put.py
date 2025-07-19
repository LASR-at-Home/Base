#!/usr/bin/env python
from unicodedata import name
import rospy
import smach
import smach_ros

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header  
from lasr_skills import ReceiveObject, PlayMotion, GoToLocation, Say
from lasr_manipulation_msgs.msg import PlaceAction, PlaceGoal
import yaml
import rosparam
from smach import CBState
# from tasks.lift.src.lift import sm


class HandoverAndDeliver(smach.StateMachine):
    def __init__(self):
        super(HandoverAndDeliver, self).__init__(
            outcomes=["succeeded", "failed", "escape"],
            input_keys=[
                "selected_object",
                "place_pose",
                "surface_id",
                "goal_pose",
            ],
        )

        with self:
            smach.StateMachine.add(
                "RECEIVE_OBJECT",
                ReceiveObject(object_name="kuat"),
                transitions={
                    "succeeded": "LOAD_PLACE_POSE",
                    "failed": "failed",
                },
            )

            # detection obj
            smach.StateMachine.add(
                "LOAD_PLACE_POSE",
                LoadPlacePoseByName(),
                transitions={
                    "succeeded": "GO_TO_LOCATION",
                    "failed": "failed",
                },
                
            )

            # go to location
            smach.StateMachine.add(
                "GO_TO_LOCATION",
                GoToLocation(),
                transitions={
                    "succeeded": "PLAY_PREPLACE_MOTION",
                    "failed": "PLAY_PREPLACE_MOTION",
                },
                remapping={"location": "pose"},
            )

            
            smach.StateMachine.add(
                "PLAY_PREPLACE_MOTION",
                PlayMotion(motion_name="home_to_preplace"),
                transitions={
                    "succeeded": "PLACE_OBJECT",
                    "preempted": "failed",
                    "aborted": "failed",
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
                    "aborted": "HELP_PLACE",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "HELP_PLACE",
                Say(
                    text="Referee, I am unable to place. Could you please place the object for me?"
                ),
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
                    "succeeded": "WAIT_AFTER_GRIPPER",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )
            smach.StateMachine.add(
                "WAIT_AFTER_GRIPPER",
                CBState(lambda ud: rospy.sleep(5.0) or "succeeded", outcomes=["succeeded"]),
                transitions={"succeeded": "HOME_POSITION"},
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
        goal.object_id = userdata.selected_object
        goal.candidate_poses = [userdata.place_pose] 
        goal.surface_id = userdata.surface_id
        return goal


if __name__ == "__main__":
    rospy.init_node("handover_and_deliver_test")

    # Define goal location
    goal_pose_stamped = PoseStamped(
        header=Header(frame_id="map"),
        pose=Pose(
            position=Point(x=0.7, y=-0.15, z=0.95),
            orientation=Quaternion(x=0.600, y=0.100, z=0.048, w=0.749),
        ),
    )

    sm = HandoverAndDeliver(location_pose=goal_pose_stamped.pose)

    sm.userdata.selected_object = type("Obj", (), {"name": "object_1"})()
    sm.userdata.object_name = sm.userdata.selected_object.name

    sm.userdata.place_pose = PoseStamped(
        header=Header(frame_id="base_footprint"),
        pose=Pose(
            position=Point(x=0.7, y=-0.15, z=0.95),
            orientation=Quaternion(w=1.0),
        ),
    )
    sm.userdata.goal_pose = goal_pose_stamped  
    sm.userdata.surface_id = "shelf"
    outcome = sm.execute()
    rospy.loginfo(f"State machine finished with outcome: {outcome}")