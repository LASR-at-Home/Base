#!/usr/bin/env python3
import os
import smach
import rospy
import rospkg
import sys
from typing import Dict
from gpsr.load_known_data import GPSRDataLoader
from gpsr.state_machine_factory import build_state_machine
from gpsr.regex_command_parser import Configuration
from gpsr.states import CommandParserStateMachine
from gpsr.regex_command_parser import gpsr_compile_and_parse

import actionlib
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pal_interaction_msgs.msg import TtsGoal, TtsAction

from lasr_skills import GoToLocation, DetectDoorOpening, Say, ListenForWakeword


def load_gpsr_configuration() -> Configuration:
    gpsr_data_dir = os.path.join(
        rospkg.RosPack().get_path("gpsr"), "data", "salvador_data"
    )
    """Loads the configuration for the GPSR command parser"""
    data_loader = GPSRDataLoader(data_dir=gpsr_data_dir)
    gpsr_known_data: Dict = data_loader.load_data()
    config = Configuration(
        {
            "person_names": gpsr_known_data["names"],
            "location_names": gpsr_known_data["non_placeable_locations"],
            "placement_location_names": gpsr_known_data["placeable_locations"],
            "room_names": gpsr_known_data["rooms"],
            "object_names": gpsr_known_data["objects"],
            "object_categories_plural": gpsr_known_data["categories_plural"],
            "object_categories_singular": gpsr_known_data["categories_singular"],
        }
    )
    return config


def _move_base(client: actionlib.SimpleActionClient, pose: PoseStamped) -> MoveBaseGoal:
    goal: MoveBaseGoal = MoveBaseGoal()
    goal.target_pose = pose
    client.send_goal(goal)
    client.wait_for_result()

    return goal


def _tts(client: actionlib.SimpleActionClient, text: str) -> None:
    tts_goal: TtsGoal = TtsGoal()
    tts_goal.rawtext.text = text
    tts_goal.rawtext.lang_id = "en_GB"
    client.send_goal(tts_goal)
    client.wait_for_result()


class Start(smach.StateMachine):

    def __init__(self):

        super().__init__(outcomes=["succeeded"])

        with self:

            smach.StateMachine.add(
                "WAIT_FOR_DOOR_TO_OPEN",
                DetectDoorOpening(),
                transitions={"door_opened": "GO_TO_INSTRUCTION_POINT"},
            )


class GoToInstructionPoint(smach.StateMachine):

    def __init__(self):

        super.__init__(self, outcomes=["succeeded"])

        instruction_pose_param = rospy.get_param("gpsr/arena/start_pose")
        instruction_pose = PoseStamped(
            pose=Pose(
                position=Point(**instruction_pose_param["position"]),
                orientation=Quaternion(**instruction_pose_param["orientation"]),
            )
        )
        instruction_pose.header.frame_id = "map"

        smach.StateMachine.add(
            "SAY_GOING_TO_START",
            Say(text="I am going to the instruction point to receive a command."),
            transitions={
                "succeeded": "GO_TO_START",
                "preempted": "GO_TO_START",
                "aborted": "GO_TO_START",
            },
        )

        smach.StateMachine.add(
            "GO_TO_START",
            GoToLocation(location=instruction_pose),
            transitions={"succeeded": "succeeded", "failed": "GO_TO_START"},
        )


class WaitForWakeword(smach.StateMachine):

    def __init__(self, timeout: float):

        super().__init__(self, outcomes=["succeeded"])

        with self:

            smach.StateMachine.add(
                "SAY_HI_TIAGO",
                Say(
                    text="Please say 'hi tiago' when you are ready to give me your command."
                ),
                transitions={
                    "succeeded": "LISTEN_FOR_HI_TIAGO",
                    "preempted": "LISTEN_FOR_HI_TIAGO",
                    "aborted": "LISTEN_FOR_HI_TIAGO",
                },
            )

            smach.StateMachine.add(
                "LISTEN_FOR_HI_TIAGO",
                ListenForWakeword(wakeword="hi_tiago", timeout=timeout),
                transitions={
                    "succeeded": "succeeded",
                    "preempted": "succeeded",
                    "aborted": "succeeded",
                },
            )


def main() -> None:
    N_COMMANDS: int = 3
    config = load_gpsr_configuration()
    tts_client = actionlib.SimpleActionClient("tts", TtsAction)
    tts_client.wait_for_server()
    _tts(tts_client, "Please open the door")
    timeout = 10.0
    start_sm = Start()
    start_sm.execute()
    rospy.sleep(3)
    for i in range(N_COMMANDS):
        rospy.loginfo(f"Command {i + 1}")
        go_to_sm = GoToInstructionPoint()
        go_to_sm.execute()
        try:
            rospy.loginfo(f"Starting GPSR")
            wakeword_sm = WaitForWakeword(timeout)
            wakeword_sm.execute()
            timeout = 120.0
            command_parser_sm = CommandParserStateMachine(data_config=config)
            command_parser_sm.execute()
            parsed_command: Dict = command_parser_sm.userdata.parsed_command
            rospy.loginfo(f"Parsed command: {parsed_command}")
            sm = build_state_machine(parsed_command)
            sm.execute()
        except:
            _tts(tts_client, "Something went wrong, I couldn't execute the command")
    _tts(tts_client, "I am done")


if __name__ == "__main__":
    rospy.init_node("gpsr_main")
    main()
    rospy.spin()
