import smach
from lasr_skills import Listen
from lasr_skills import Say

from typing import Union

'''
class AskAndListen(smach.StateMachine):
    def __init__(
        self,
        tts_phrase: Union[str, None] = None,
        tts_phrase_format_str: Union[str, None] = None,
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["transcribed_speech"],
        )
        # if tts_phrase is not None:
        #     smach.StateMachine.__init__(
        #         self,
        #         outcomes=["succeeded", "failed"],
        #         output_keys=["transcribed_speech"],
        #     )
        #     with self:
        #         smach.StateMachine.add(
        #             "SAY",
        #             Say(text=tts_phrase),
        #             transitions={
        #                 "succeeded": "LISTEN",
        #                 "aborted": "failed",
        #                 "preempted": "failed",
        #             },
        #         )
        #         smach.StateMachine.add(
        #             "LISTEN",
        #             Listen(),
        #             transitions={
        #                 "succeeded": "succeeded",
        #                 "aborted": "failed",
        #                 "preempted": "failed",
        #             },
        #             remapping={"sequence": "transcribed_speech"},
        #         )
        # elif tts_phrase_format_str is not None:
        #     smach.StateMachine.__init__(
        #         self,
        #         outcomes=["succeeded", "failed"],
        #         output_keys=["transcribed_speech"],
        #         input_keys=["tts_phrase_placeholders"],
        #     )
        #     with self:
        #         smach.StateMachine.add(
        #             "SAY",
        #             Say(format_str=tts_phrase_format_str),
        #             transitions={
        #                 "succeeded": "LISTEN",
        #                 "aborted": "failed",
        #                 "preempted": "failed",
        #             },
        #             remapping={"placeholders": "tts_phrase_placeholders"},
        #         )
        #         smach.StateMachine.add(
        #             "LISTEN",
        #             Listen(),
        #             transitions={
        #                 "succeeded": "succeeded",
        #                 "aborted": "failed",
        #                 "preempted": "failed",
        #             },
        #             remapping={"sequence": "transcribed_speech"},
        #         )

        # else:
        #     smach.StateMachine.__init__(
        #         self,
        #         outcomes=["succeeded", "failed"],
        #         output_keys=["transcribed_speech"],
        #         input_keys=["tts_phrase"],
        #     )
        #     with self:
        #         smach.StateMachine.add(
        #             "SAY",
        #             Say(),
        #             transitions={
        #                 "succeeded": "LISTEN",
        #                 "aborted": "failed",
        #                 "preempted": "failed",
        #             },
        #             remapping={"text": "tts_phrase"},
        #         )
        #         smach.StateMachine.add(
        #             "LISTEN",
        #             Listen(),
        #             transitions={
        #                 "succeeded": "succeeded",
        #                 "aborted": "failed",
        #                 "preempted": "failed",
        #             },
        #             remapping={"sequence": "transcribed_speech"},
        #         )
        pass
    
    def execute(self, userdata):
        userdata.transcribed_speech = "Tell me how many people in the kitchen are wearing white shirts"

        return "succeeded"
'''
import sys
import os
sys.path.insert(0, os.path.expanduser(
    '~/Documents/Robocup/robocup_ws/CommandGenerator/src'
))

import rospkg
import os
from gpsr.load_known_data import GPSRDataLoader
from robocupathome_generator.gpsr_commands import CommandGenerator
from gpsr.regex_command_parser import Configuration


class AskAndListen(smach.StateMachine):
    def __init__(
        self,
        tts_phrase: Union[str, None] = None,
        tts_phrase_format_str: Union[str, None] = None,
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["transcribed_speech"],
        )

        # -- load the same GPSR data folder used elsewhere --
        data_dir = os.path.join(
            rospkg.RosPack().get_path("gpsr"),
            "data",
            "eindhoven_data"
        )
        loader = GPSRDataLoader(data_dir=data_dir)
        gpsr_data = loader.load_data()

        # -- build a CommandGenerator just like the CLI does --
        self._cmdgen = CommandGenerator(
            gpsr_data["names"],
            gpsr_data["non_placeable_locations"],
            gpsr_data["placeable_locations"],
            gpsr_data["rooms"],
            gpsr_data["objects"],
            gpsr_data["categories_plural"],
            gpsr_data["categories_singular"],
        )

    def execute(self, userdata):
        #cmd = self._cmdgen.generate_command_start(cmd_category="people")
        #cmd = "tell me how many waving persons are in the kitchen"
        #cmd = "tell me how many people in the hallway are wearing blue t shirts"
        #cmd = "Count spoons there are on the kitchen counter"
        cmd = "tell me how many spoons there are on the kitchen counter"
        #cmd = "Guide the person wearing a purple t shirt from the kitchen cabinet to the dinner table"
        userdata.transcribed_speech = cmd

        return "succeeded"
