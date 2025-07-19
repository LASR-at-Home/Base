#!/usr/bin/env python
import rospy
import smach
import smach_ros
import random
import re

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header  
from lasr_skills import ReceiveObject, PlayMotion, GoToLocation, Say
from lasr_manipulation_msgs.msg import PlaceAction, PlaceGoal
from smach import CBState

# Predefined fallback locations
location_dict = {
    "table": "table",
    "sink": "sink",
    "shelf": "shelf",
    # "fridge": "fridge",
    "dishwasher": "dishwasher",
}


class HandoverAndDeliver(smach.StateMachine):
    def __init__(self):
        super(HandoverAndDeliver, self).__init__(
            outcomes=["succeeded", "failed", "escape"],
            input_keys=[
                "selected_object",
                "place_pose",
                "surface_id",
                "goal_pose",
                "place_location_name",
            ],
        )

        with self:
            smach.StateMachine.add(
                "RECEIVE_OBJECT",
                ReceiveObject(),
                transitions={
                    "succeeded": "LOAD_PLACE_POSE",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "LOAD_PLACE_POSE",
                LoadPlacePoseByName(),
                transitions={
                    "succeeded": "GO_TO_LOCATION",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "GO_TO_LOCATION",
                GoToLocation(),
                transitions={
                    "succeeded": "SAY_PREPLACE",
                    "failed": "SAY_PREPLACE",
                },
                remapping={"location": "pose"},
            )

            smach.StateMachine.add(
                "SAY_PREPLACE",
                Say(text="I am going to move my arm to the pre-place position. Please keep clear of me and be ready to press the emergency stop."),
                transitions={
                    "succeeded": "PLAY_PREPLACE_MOTION",
                    "preempted" : "PLAY_PREPLACE_MOTION",
                    "aborted" : "PLAY_PREPLACE_MOTION"
                }
            )

            smach.StateMachine.add(
                "PLAY_PREPLACE_MOTION",
                PlayMotion(motion_name="home_to_preplace"),
                transitions={
                    "succeeded": "SAY_MOVING_FORWARD",
                    "preempted": "SAY_MOVING_FORWARD",
                    "aborted": "SAY_MOVING_FORWARD",
                },
            )

            smach.StateMachine.add(
                "SAY_MOVING_FORWARD",
                Say(text="I'm moving forward to the place position. Keep clear."),
                transitions={
                    "succeeded": "MOVING_FORWARD",
                    "preempted" : "MOVING_FORWARD",
                    "aborted" : "MOVING_FORWARD"
                }
            )

            smach.StateMachine.add(
                "MOVING_FORWARD",
                GoToLocation(),
                transitions={
                    "succeeded": "SAY_PLACING",
                    "failed": "SAY_PLACING",
                },
                remapping={"location": "pre_place_pose"},
            )

            smach.StateMachine.add(
                "SAY_PLACING",
                Say(text="I'm now placing the object. Keep clear and be ready to press the emergency stop."),
                transitions={
                    "succeeded": "PLACE_OBJECT",
                    "preempted" : "PLACE_OBJECT",
                    "aborted" : "PLACE_OBJECT"
                }
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
                    text="Referee, I am unable to place. Could you please place the object for me?. I will open my gripper in 5.. 4... 3... 2.. 1.."
                ),
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
                    "succeeded": "WAIT_AFTER_GRIPPER",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "WAIT_AFTER_GRIPPER",
                CBState(lambda ud: rospy.sleep(10.0) or "succeeded", outcomes=["succeeded"]),
                transitions={"succeeded": "SAY_MOVING_ARM_AGAIN"},
            )

            smach.StateMachine.add(
                "SAY_MOVING_ARM_AGAIN",
                Say(
                    text="I am now going to move my arm back to the pre-place position. Watch out."
                ),
                transitions={
                    "succeeded": "PLAY_PREPLACE_MOTION_2",
                    "preempted": "PLAY_PREPLACE_MOTION_2",
                    "aborted": "PLAY_PREPLACE_MOTION_2",
                },
            )

            smach.StateMachine.add(
                "PLAY_PREPLACE_MOTION_2",
                PlayMotion(motion_name="home_to_preplace"),
                transitions={
                    "succeeded": "SAY_DONE",
                    "preempted": "SAY_DONE",
                    "aborted": "SAY_DONE",
                },
            )

            smach.StateMachine.add(
                "SAY_DONE",
                Say(
                    text="I am done."
                ),
                transitions={
                    "succeeded": "succeeded",
                    "preempted": "succeeded",
                    "aborted": "succeeded",
                },
            )

            # smach.StateMachine.add(
            #     "HOME_POSITION",
            #     PlayMotion(motion_name="home"),
            #     transitions={
            #         "succeeded": "succeeded",
            #         "preempted": "failed",
            #         "aborted": "failed",
            #     },
            # )

    def _place_cb(self, userdata, goal):
        goal = PlaceGoal()
        goal.object_id = userdata.selected_object
        goal.candidate_poses = [userdata.place_pose]
        goal.surface_id = userdata.surface_id
        return goal


class LoadPlacePoseByName(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["place_location_name"],
            output_keys=["place_pose", "pose", "surface_id", "selected_object", "place_location_name", "pre_place_pose"],
        )

    def execute(self, userdata):
        name = userdata.place_location_name
        # name = getattr(userdata, "place_location_name", None)

        # If not valid, select random location
        if not name or name not in location_dict:
            rospy.logwarn("[LoadPlacePoseByName] No valid location name passed. Selecting random location.")
            name = random.choice(list(location_dict.keys()))
            userdata.place_location_name = name  # Optional: store back

        rospy.loginfo(f"[LoadPlacePoseByName] Looking for location: {name}")

        try:
            location_pose = rospy.get_param(f"give_me_a_hand/{name}/pose")
            place_pose_raw = rospy.get_param(f"give_me_a_hand/{name}/place_pose")
            pre_place_pose_raw = rospy.get_param(f"give_me_a_hand/{name}/pre_place_pose")
        except KeyError as e:
            rospy.logerr(f"[LoadPlacePoseByName] Missing ROS param: {e}")
            return "failed"

        userdata.pose = Pose(
            position=Point(**location_pose["position"]),
            orientation=Quaternion(**location_pose["orientation"]),
        )

        userdata.place_pose = PoseStamped(
            header=Header(frame_id="map"),
            pose=Pose(
                position=Point(**place_pose_raw["position"]),
                orientation=Quaternion(**place_pose_raw["orientation"]),
            ),
        )

        userdata.pre_place_pose = Pose(
            position=Point(**pre_place_pose_raw["position"]),
            orientation=Quaternion(**pre_place_pose_raw["orientation"]),
        )
        userdata.surface_id = "table"
        userdata.selected_object = "object_1"

        return "succeeded"


if __name__ == "__main__":
    rospy.init_node("handover_and_deliver_test")
    sm = HandoverAndDeliver()

    # Optionally comment this line to test random fallback
    sm.userdata.place_location_name = 'table'

    outcome = sm.execute()
    rospy.loginfo(f"[Main] State machine finished with outcome: {outcome}")
