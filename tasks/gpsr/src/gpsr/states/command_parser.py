#!/usr/bin/env python3
import smach
import rospy

from gpsr.regex_command_parser import Configuration, gpsr_compile_and_parse
from gpsr.states import CommandSimilarityMatcher
from lasr_skills import AskAndListen, Say
from lasr_skills.vision import GetImage

import cv2_img
import cv2


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
            input_keys=["matched_command"],
            output_keys=["parsed_command"],
        )
        self.data_config = data_config

    def execute(self, userdata):
        rospy.loginfo(f"Received command : {userdata.matched_command.lower()}")
        try:
            print(userdata.matched_command.lower())
            userdata.parsed_command = gpsr_compile_and_parse(
                self.data_config, userdata.matched_command.lower()
            )
        except Exception as e:
            rospy.logerr(e)
            return "failed"
        return "succeeded"


class CommandParserStateMachine(smach.StateMachine):

    class CheckResponse(smach.State):

        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["correct", "incorrect"],
                input_keys=["transcribed_speech"],
            )

        def execute(self, userdata):
            if "yes" in userdata.transcribed_speech.lower():  # TODO: make this smarter
                return "correct"
            return "incorrect"

    class QRCodeToCommand(smach.StateMachine):

        class DetectQRCode(smach.State):

            _qr_code_detector: cv2.QRCodeDetector

            def __init__(self):
                smach.State.__init__(
                    self,
                    outcomes=["succeeded", "failed"],
                    input_keys=["img_msg"],
                    output_keys=["qr_code_text"],
                )

                self._qr_code_detector = cv2.QRCodeDetector()

            def execute(self, userdata):
                cv_im = cv2_img.msg_to_cv2_img(userdata.img_msg)
                text, _, _ = self._qr_code_detector.detectAndDecode(cv_im)
                if text:
                    userdata.qr_code_text = text
                    return "succeeded"
                return "failed"

        def __init__(self):
            smach.StateMachine.__init__(
                self, outcomes=["succeeded", "failed"], output_keys=["qr_code_text"]
            )

            with self:

                smach.StateMachine.add(
                    "GET_IMAGE",
                    GetImage(),
                    transitions={"succeeded": "DETECT_QR_CODE", "failed": "failed"},
                )

                smach.StateMachine.add(
                    "DETECT_QR_CODE",
                    self.DetectQRCode(),
                    transitions={"succeeded": "succeeded", "failed": "GET_IMAGE"},
                )

    def __init__(
        self,
        data_config: Configuration,
        n_vecs_per_txt_file: int = 897824,
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
        self._command_counter = 0
        self._command_counter_threshold = 3

        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        with self:

            smach.StateMachine.add(
                "ASK_FOR_COMMAND",
                AskAndListen(
                    tts_phrase="Hello, please tell me your command. Speak closely and directly into the microphone "
                ),
                transitions={
                    "succeeded": "COMMAND_SIMILARITY_MATCHER",
                    "failed": "ASK_FOR_COMMAND",
                },
                remapping={"transcribed_speech": "raw_command"},
            )

            smach.StateMachine.add(
                "COMMAND_SIMILARITY_MATCHER",
                CommandSimilarityMatcher([n_vecs_per_txt_file] * total_txt_files),
                transitions={"succeeded": "SAY_COMMAND", "failed": "failed"},
                remapping={
                    "command": "raw_command",
                },
            )

            smach.StateMachine.add(
                "SAY_COMMAND",
                Say(format_str="I received command: {}. "),
                transitions={
                    "succeeded": "ASK_IF_CORRECT",
                    "preempted": "ASK_IF_CORRECT",
                    "aborted": "ASK_IF_CORRECT",
                },
                remapping={"placeholders": "matched_command"},
            )

            smach.StateMachine.add(
                "ASK_IF_CORRECT",
                AskAndListen(
                    tts_phrase="Is this correct? Please Say Hi Tiago Yes or Hi Tiago No."
                ),
                transitions={
                    "succeeded": "CHECK_RESPONSE",
                    "failed": "CHECK_RESPONSE",
                },
            )

            smach.StateMachine.add(
                "CHECK_RESPONSE",
                self.CheckResponse(),
                transitions={
                    "correct": "PARSE_COMMAND",
                    "incorrect": "failed",
                },
            )

            smach.StateMachine.add(
                "PARSE_COMMAND",
                ParseCommand(data_config),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
                remapping={"parsed_command": "parsed_command"},
            )

    def _increment_counter(self, userdata):
        """Increment counter and determine whether command has been parsed over the number of times
        set by the threshold.
        """
        self._command_counter += 1
        if self._command_counter >= self._command_counter_threshold:
            self._command_counter = 0
            return "exceeded"
        return "continue"
