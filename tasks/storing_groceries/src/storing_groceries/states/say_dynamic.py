import smach
import smach_ros
import rospy
import os
from typing import Union


# Detect environment
PUBLIC_CONTAINER: bool = False
try:
    from pal_interaction_msgs.msg import TtsGoal, TtsAction, TtsText
except ImportError:
    PUBLIC_CONTAINER = True

SIMULATION: bool = "tiago" not in os.environ.get("ROS_MASTER_URI", "")

# Define Say class based on environment
if PUBLIC_CONTAINER or SIMULATION:

    class SayDynamic(smach.State):
        def __init__(
            self,
            text: Union[str, None] = None,
            format_str: Union[str, None] = None,
            text_fn=None
        ):
            input_keys = []
            if format_str is not None:
                input_keys = ["placeholders"]
            elif text_fn is not None:
                input_keys = ["table_object", "table_object_category", "cabinet_categories", "cabinet_num"]  # Dummy to allow full userdata access

            super(SayDynamic, self).__init__(
                outcomes=["succeeded", "aborted", "preempted"],
                input_keys=input_keys,
            )

            self.text = text
            self.format_str = format_str
            self.text_fn = text_fn

            if PUBLIC_CONTAINER:
                rospy.logwarn("SayDynamic: Public container detected; speech will not be executed.")
            elif SIMULATION:
                rospy.logwarn("SayDynamic: Simulation detected; speech will not be executed.")

        def execute(self, userdata):
            if self.text is not None:
                rospy.loginfo(f"SayDynamic: {self.text}")

            elif self.format_str is not None:
                try:
                    msg = self.format_str.format(userdata.placeholders)
                    rospy.loginfo(f"SayDynamic: {msg}")
                except Exception as e:
                    rospy.logerr(f"SayDynamic format error: {e}")
                    return "aborted"

            elif self.text_fn is not None:
                try:
                    msg = self.text_fn(userdata)
                    rospy.loginfo(f"SayDynamic: {msg}")
                except Exception as e:
                    rospy.logerr(f"SayDynamic dynamic text_fn error: {e}")
                    return "aborted"

            else:
                try:
                    rospy.loginfo(f"SayDynamic: {userdata.text}")
                except Exception as e:
                    rospy.logerr(f"SayDynamic fallback error: {e}")
                    return "aborted"

            return "succeeded"

else:

    class SayDynamic(smach_ros.SimpleActionState):
        def __init__(
            self,
            text: Union[str, None] = None,
            format_str: Union[str, None] = None,
            text_fn=None
        ):
            if text is not None:
                super().__init__(
                    "tts",
                    TtsAction,
                    goal=TtsGoal(
                        rawtext=TtsText(text=text, lang_id="en_GB")
                    ),
                )

            elif format_str is not None:
                super().__init__(
                    "tts",
                    TtsAction,
                    goal_cb=lambda ud, _: TtsGoal(
                        rawtext=TtsText(
                            text=format_str.format(*ud.placeholders)
                            if isinstance(ud.placeholders, (list, tuple))
                            else format_str.format(ud.placeholders),
                            lang_id="en_GB"
                        )
                    ),
                    input_keys=["placeholders"],
                )

            elif text_fn is not None:
                super().__init__(
                    "tts",
                    TtsAction,
                    goal_cb=lambda ud, _: TtsGoal(
                        rawtext=TtsText(
                            text=text_fn(ud),
                            lang_id="en_GB"
                        )
                    ),
                    input_keys = ["table_object", "table_object_category", "cabinet_categories", "cabinet_num"]  # Dummy to allow full userdata access
                )

            else:
                super().__init__(
                    "tts",
                    TtsAction,
                    goal_cb=lambda ud, _: TtsGoal(
                        rawtext=TtsText(
                            text=ud.text,
                            lang_id="en_GB"
                        )
                    ),
                    input_keys=["text"],
                )
