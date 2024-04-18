import smach_ros

from pal_interaction_msgs.msg import TtsGoal, TtsAction, TtsText

from typing import Union


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
                            text=format_str.format(*ud.placeholders), lang_id="en_GB"
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
