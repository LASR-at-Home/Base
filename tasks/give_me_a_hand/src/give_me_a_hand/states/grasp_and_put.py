#!/usr/bin/env python
import rospy
import smach
import smach_ros

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header  
from lasr_skills import ReceiveObject, PlayMotion, GoToLocation
from lasr_manipulation_msgs.msg import PlaceAction, PlaceGoal
import yaml
import rosparam
from tasks.lift.src.lift import sm


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
                ReceiveObject(),
                transitions={
                    "succeeded": "PLAY_PREPLACE_MOTION",
                    "failed": "failed",
                },
            )

            # go to location
            smach.StateMachine.add(
                "DELIVER_OBJECT",
                GoToLocation(),
                transitions={
                    "succeeded": "PLAY_PREPLACE_MOTION",
                    "failed": "PLAY_PREPLACE_MOTION",
                },
                remapping={"goal_pose": "goal_pose"},
            )

            # detection obj
            smach.StateMachine.add(
                "LOAD_PLACE_POSE",
                LoadPlacePoseByName(),
                transitions={
                    "succeeded": "PLAY_PREPLACE_MOTION",
                    "failed": "failed",
                },
                
            )

            smach.StateMachine.add(
                "GO_TO_LOCATION",
                GoToLocation(location_param = "give_me_a_hand/place_location"),
                transitions={
                    "succeeded": "DETECT",
                    "failed": "failed",
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
        goal.object_id = userdata.obj_id
        goal.candidate_poses = [userdata.place_pose] 
        goal.surface_id = userdata.surface_id
        return goal
    

class LoadPlacePoseByName(smach.State):
    def __init__(self, yaml_file_path):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["place_location_name"],
            output_keys=["place_pose"],
        )
        with open(yaml_file_path, 'r') as f:
            self.pose_dict = yaml.safe_load(f)

    def execute(self, userdata):
        name = userdata.place_location_name
        rospy.loginfo(f"[LoadPlacePoseByName] Looking for location: {name}")

        if name not in self.pose_dict:
            rospy.logerr(f"[LoadPlacePoseByName] '{name}' not found in YAML.")
            return "failed"
        
        userdata.pose = rospy.get_param(
            f"give_me_a_have/{name}/pose", None
        )

        userdata.place_pose = rospy.get_param(
            f"give_me_a_have/{name}/place_pose", None
        )

        userdata.surface_id = "shelf"
        userdata.obj_id = "object_1"

        return "succeeded"



if __name__ == "__main__":
    rospy.init_node("handover_and_deliver_test")

    # Define goal location
    sm.userdata.place_location_name = "shelf"
    sm = HandoverAndDeliver()
    outcome = sm.execute()







