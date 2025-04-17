import smach_ros
import smach
import rclpy
import os
from lasr_skills import AccessNode
from typing import Union

PUBLIC_CONTAINER: bool = False

class SayTemp(smach.State):

            
    text: Union[str, None] = None
    format_str: Union[str, None] = None

    def __init__(
        self, text: Union[str, None] = "None", format_str: Union[str, None] = None
    ):
        self.node = AccessNode.get_node()
        if text is not None:
            super(SayTemp, self).__init__(
                outcomes=["succeeded", "aborted", "preempted"]
            )
        elif format_str is not None:
            super(SayTemp, self).__init__(
                outcomes=["succeeded", "aborted", "preempted"],
                input_keys=["placeholders"],
            )
        else:
            super(SayTemp, self).__init__(
                outcomes=["succeeded", "aborted", "preempted"],
                input_keys=["text"],
            )

        self.text = text
        self.format_str = format_str

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
        

def main(args=None):
    rclpy.init(args=args)

    sm = smach.StateMachine(outcomes=["preempted", "succeeded", "aborted"])
    with sm:
        smach.StateMachine.add(
            "Say",
            SayTemp(),
            transitions={
                "preempted": "preempted",
                "succeeded": "succeeded",
                "aborted": "aborted",
            },
        )

    outcome = sm.execute()


if __name__ == "__main__":
    main()
