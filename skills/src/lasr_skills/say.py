#!/usr/bin/env python3
import yasmin
import rclpy
import os

HAS_TTS_MSGS: bool = True


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
            super().__init__(
                outcomes=["succeeded", "aborted", "preempted"]
            )

            self.text = text
            self.format_str = format_str
            self.node.get_logger().info(
                "tts_msgs not available, the Say skill will not work."
            )

        def execute(self, blackboard):
            self.node.get_logger().info(self.text)
            return "succeeded"

else:

    class Say(yasmin_ros.SimpleActionState):
        def __init__(
            self,
            node: rclpy.node.Node,
            text: Union[str, None] = None,
            format_str: Union[str, None] = None,
        ):
            if text is not None:
                super().__init__(
                    node=node,
                    action_name="/tts_engine/tts",
                    action_spec=TTS,
                    goal=TTS.Goal(input=text),
                )
            elif format_str is not None:
                super().__init__(
                    node,
                    "/tts_engine/tts",
                    TTS,
                    goal_cb=lambda ud, _: (
                        TTS.Goal(
                            input=format_str.format(*ud.placeholders),
                        )
                        if isinstance(ud.placeholders, (list, tuple))
                        else TTS.Goal(input=format_str.format(ud.placeholders))
                    ),
                    input_keys=["placeholders"],
                )
            else:
                super().__init__(
                    node,
                    "/tts_engine/tts",
                    TTS,
                    goal_cb=lambda ud, _: TTS.Goal(input=ud.text),
                    input_keys=["text"],
                )


class SayStateNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("say_state_node")

        sm = smach.StateMachine(outcomes=["succeeded", "failed"])

        with sm:
            smach.StateMachine.add(
                "SAY",
                Say(self, text="Hello"),
                transitions={
                    "succeeded": "succeeded",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

        outcome = sm.execute()


def main(args=None):
    rclpy.init(args=args)
    node = SayStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
