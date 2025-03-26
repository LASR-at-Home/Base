#!/usr/bin/env python3
import smach_ros
import smach
import rclpy
import os
from lasr_skills import AccessNode

HAS_TTS_MSGS: bool = False


try:
    from tts_msgs.msg import TtsGoal, TtsAction, TtsText
except ImportError:
    HAS_TTS_MSGS = True


from typing import Union

if not HAS_TTS_MSGS:

    class Say(smach.State):

        text: Union[str, None] = None
        format_str: Union[str, None] = None

        def __init__(
            self, text: Union[str, None] = "None", format_str: Union[str, None] = None
        ):
            self.node = AccessNode.get_node()
            if text is not None:
                super(Say, self).__init__(
                    outcomes=["succeeded", "aborted", "preempted"]
                )
            elif format_str is not None:
                super(Say, self).__init__(
                    outcomes=["succeeded", "aborted", "preempted"],
                    input_keys=["placeholders"],
                )
            else:
                super(Say, self).__init__(
                    outcomes=["succeeded", "aborted", "preempted"],
                    input_keys=["text"],
                )

            self.text = text
            self.format_str = format_str
            self.node.get_logger().info(
                "tts_msgs not available, the Say skill will not work."
            )

        def execute(self, userdata):
            if self.text is not None:
                self.node.get_logger().info(self.text)
            elif self.format_str is not None:
                self.node.get_logger().info(
                    self.format_str.format(userdata.placeholders)
                )
            else:
                self.node.get_logger().info(userdata.text)
            return "succeeded"

else:

    class Say(smach_ros.SimpleActionState):
        def __init__(
            self, text: Union[str, None] = None, format_str: Union[str, None] = None
        ):
            self.node = AccessNode.get_node()
            if text is not None:
                super(Say, self).__init__(
                    "tts",
                    TtsAction,
                    goal=TtsGoal(rawtext=TtsText(text=text, lang_id="en_GB")),
                )
            elif format_str is not None:
                super(Say, self).__init__(
                    "tts",
                    TtsAction,
                    goal_cb=lambda ud, _: (
                        TtsGoal(
                            rawtext=TtsText(
                                text=format_str.format(*ud.placeholders),
                                lang_id="en_GB",
                            )
                        )
                        if isinstance(ud.placeholders, (list, tuple))
                        else TtsGoal(
                            rawtext=TtsText(
                                text=format_str.format(ud.placeholders), lang_id="en_GB"
                            )
                        )
                    ),
                    input_keys=["placeholders"],
                )
            else:
                super(Say, self).__init__(
                    "tts",
                    TtsAction,
                    goal_cb=lambda ud, _: TtsGoal(
                        rawtext=TtsText(text=ud.text, lang_id="en_GB")
                    ),
                    input_keys=["text"],
                )


def main(args=None):
    rclpy.init(args=args)

    sm = smach.StateMachine(outcomes=["preempted", "succeeded", "aborted"])
    with sm:
        smach.StateMachine.add(
            "Say",
            Say(),
            transitions={
                "preempted": "preempted",
                "succeeded": "succeeded",
                "aborted": "aborted",
            },
        )

    outcome = sm.execute()


if __name__ == "__main__":
    main()
