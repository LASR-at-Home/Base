#!/usr/bin/env python3
import smach
import rospy

from gpsr.regex_command_parser import Configuration, gpsr_compile_and_parse
from gpsr.states import CommandSimilarityMatcher
from lasr_skills import AskAndListen


class ParseCommand(smach.State):
    def __init__(self, data_config: Configuration):
        """Takes in a string containing the command and runs the command parser
        that outputs a dictionary of parameters for the command.

        Args:
            data_config (Configuration): Configuration object containing the regex patterns
        """
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["raw_command"],
            output_keys=["parsed_command"],
        )
        self.data_config = data_config

    def execute(self, userdata):
        rospy.loginfo(f"Received command : {userdata.raw_command.lower()}")
        try:
            userdata.parsed_command = gpsr_compile_and_parse(
                self.data_config, userdata.raw_command.lower()
            )
        except Exception as e:
            rospy.logerr(e)
            return "failed"
        return "succeeded"


class CommandParserStateMachine(smach.StateMachine):
    def __init__(
        self,
        data_config: Configuration,
        n_vecs_per_txt_file: int = 1177943,
        total_txt_files: int = 10,
    ):
        """State machine that takes in a command, matches it to a known command, and
        outputs the parsed command.

        Args:
            data_config (Configuration): Configuration object containing the regex patterns
            n_vecs_per_txt_file (int, optional): number of vectors in each gpsr txt
            file. Defaults to 1177943.
            total_txt_files (int, optional): total number of gpsr txt files. Defaults to 10.
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        with self:
            smach.StateMachine.add(
                "ASK_FOR_COMMAND",
                AskAndListen(tts_phrase="Hello, please tell me your command."),
                transitions={"succeeded": "PARSE_COMMAND", "failed": "failed"},
                remapping={"transcribed_speech": "raw_command"},
            )

            smach.StateMachine.add(
                "PARSE_COMMAND",
                ParseCommand(data_config),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "COMMAND_SIMILARITY_MATCHER",
                },
                remapping={"parsed_command": "parsed_command"},
            )

            smach.StateMachine.add(
                "COMMAND_SIMILARITY_MATCHER",
                CommandSimilarityMatcher([n_vecs_per_txt_file] * total_txt_files),
                transitions={"succeeded": "PARSE_COMMAND", "failed": "failed"},
                remapping={
                    "command": "raw_command",
                    "matched_command": "raw_command",
                },
            )
