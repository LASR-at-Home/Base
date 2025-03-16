import smach_ros
import smach
import rclpy
import os

PUBLIC_CONTAINER: bool = False

try:
    from pal_interaction_msgs.msg import TtsGoal, TtsAction, TtsText
except ImportError:
    PUBLIC_CONTAINER = True

SIMULATION: bool = "tiago"

from typing import Union

if PUBLIC_CONTAINER or SIMULATION:

    class Say(smach.State):

        text: Union[str, None] = None
        format_str: Union[str, None] = None

        def __init__(
            self, text: Union[str, None] = "None", format_str: Union[str, None] = None
        ):
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
            if PUBLIC_CONTAINER:
                rclpy.logging.get_logger("Say").warning(
                    "You are using the public container, the Say skill will not work"
                )

            elif SIMULATION:
                rclpy.logging.get_logger("Say").warning(
                    "You are using the simulation, the Say skill will not work"
                )

        def execute(self, userdata):
            if self.text is not None:
                rclpy.logging.get_logger("Say").info(self.text)
            elif self.format_str is not None:
                rclpy.logging.get_logger("Say").info(
                    self.format_str.format(userdata.placeholders)
                )
            else:
                rclpy.logging.get_logger("Say").info(userdata.text)
            return "succeeded"

else:

    class Say(smach_ros.SimpleActionState):
        def __init__(
            self, text: Union[str, None] = None, format_str: Union[str, None] = None
        ):
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
