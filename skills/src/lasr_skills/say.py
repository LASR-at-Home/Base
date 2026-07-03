#!/usr/bin/env python3
import yasmin
import yasmin_ros
from yasmin_ros import set_ros_loggers
import rclpy
import os

HAS_TTS_MSGS: bool = False


try:
    from tts_msgs.action import TTS
except ImportError:
    HAS_TTS_MSGS = False
    print("doesn't have tts msgs")


from typing import Union

if not HAS_TTS_MSGS:

    class Say(yasmin.State):

        text: Union[str, None] = None
        format_str: Union[str, None] = None

        def __init__(
            self,
            text: Union[str, None] = "None",
            format_str: Union[str, None] = None,
        ):
            super().__init__(outcomes=["succeeded", "aborted", "canceled"])

            self.text = text
            self.format_str = format_str
            yasmin.YASMIN_LOG_INFO(
                "tts_msgs not available, the Say skill will not work."
            )

        def execute(self, blackboard):
            yasmin.YASMIN_LOG_INFO(self.text)
            return "succeeded"

else:

    class Say(yasmin_ros.ActionState):
        def __init__(
            self,
            text: Union[str, None] = None,
            format_str: Union[str, None] = None,
        ):
            super().__init__(
                action_name="/tts_engine/tts",
                action_type=TTS,
                create_goal_handler=self.create_goal,
            )

            self.add_input_key("placeholders")
            self.add_input_key("text")

            self.format_str = format_str
            self.text = text

        def create_goal(self, blackboard):
            goal = TTS.Goal()
            goal.locale = "en_GB"
            if self.text is None:
                if self.format_str is not None:
                    if isinstance(blackboard["placeholders"], (list, tuple)):
                        goal.input = self.format_str.format(*blackboard["placeholders"])
                    else:
                        goal.input = self.format_str.format(blackboard["placeholders"])
                else:
                    goal.input = blackboard["text"]
            else:
                goal.input = self.text
                goal.locale = "en_GB"

            return goal


def main(args=None):
    rclpy.init(args=args)
    set_ros_loggers()
    sm = yasmin.StateMachine(outcomes=["succeeded", "failed"])
    sm.add_state(
        "SAY",
        Say(
            text="hello",
            transitions={
                "succeeded": "succeeded",
                "aborted": "failed",
                "canceled": "failed",
            },
        ),
    )
    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
