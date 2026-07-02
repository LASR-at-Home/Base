#!/usr/bin/env python3
import rclpy

import yasmin
from yasmin import StateMachine, Blackboard
import yasmin_ros
from yasmin_ros import ServiceState, ActionState


import yasmin
from yasmin import StateMachine, Blackboard
import yasmin_ros
from yasmin_ros import ServiceState, ActionState

from std_srvs.srv import Empty

from lasr_skills import Say, PlayMotion, Wait

from typing import Union


class ClearOctomap(ServiceState):
    def __init__(self):
        super().__init__(
            srv_type=Empty,
            srv_name="/clear_octomap",
            create_request_handler=self._create_request,
        )

    def _create_request(self, blackboard):
        return Empty.Request()


# PORT BACK TO YASMIN AND COPY SAME PREMISE AS RECIEVE AND ADD isheld
class HandoverObject(StateMachine):
    def __init__(self, object_name: Union[str, None] = None, vertical: bool = True, can_hold: bool=False):

        super().__init__(outcomes=["succeeded", "failed"])
        if object_name is None:
            self.add_input_key("object_name")

        self.object_name = object_name
        self.vertical = vertical 

        if can_hold:
            self.createHoldable()
        else:
            self.requestPlaceInBasket()

    
    def requestPlaceInBasket(self):
        if self.object_name is not None:
            self.add_state(
                "REQUEST",
                Say(
                    text=f"Can you please hold the {self.object_name} in my gripper. I will wait a few seconds before releasing it.",
                ),
                transitions={
                    "succeeded": "WAIT_3",
                    "aborted": "failed",
                    "canceled": "failed",
                },
            )
        else:
            self.add_state(
                "REQUEST",
                Say(
                    format_str="Can you please hold the {} in my gripper. I will wait a few seconds before releasing it.",
                ),
                transitions={
                    "succeeded": "WAIT_3",
                    "aborted": "failed",
                    "canceled": "failed",
                },
                remapping={"placeholders": "object_name"},
            )
        
        self.add_state(
            "WAIT_3",
            Wait(3),
            transitions={
                "succeeded": "OPEN_GRIPPER",
                "failed": "OPEN_GRIPPER",
            },
        )
        
        self.add_state(
            "OPEN_GRIPPER",
            PlayMotion(motion_name="open"),
            transitions={
                "succeeded": "WAIT_3_2",
                "aborted": "failed",
                "canceled": "failed",
            },
        )
        self.add_state(
            "WAIT_3_2",
            Wait(3),
            transitions={
                "succeeded": "WARN_CLOSE",
                "failed": "WARN_CLOSE",
            },
        )
        self.add_state(
            "WARN_CLOSE",
            Say(
                text=f"I will now close my gripper. Please mind your fingers.",
            ),
            transitions={
                "succeeded": "CLOSE_GRIPPER",
                "aborted": "failed",
                "canceled": "failed",
            },
        )
        self.add_state(
            "CLOSE_GRIPPER",
            PlayMotion(motion_name="close"),
            transitions={
                "succeeded": "succeeded",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

    def createHoldable(self):

        self.add_state(
            "CLEAR_OCTOMAP",
            ClearOctomap(),
            transitions={"succeeded": "LOOK_AROUND", "aborted": "failed"},
        )

        self.add_state(
            "LOOK_AROUND",
            PlayMotion(motion_name="head_tour"),
            transitions={
                "succeeded": "SAY_REACH_ARM",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "SAY_REACH_ARM",
            Say(
                text="Please step back, I am going to reach my arm out."
            ),
            transitions={
                "succeeded": "REACH_ARM",
                "aborted": "REACH_ARM",
                "canceled": "REACH_ARM",
            },
        )

        if self.vertical:
            self.add_state(
                "REACH_ARM",
                PlayMotion(motion_name="reach_arm_vertical_gripper"),
                transitions={
                    "succeeded": "SAY_GRAB",
                    "aborted": "failed",
                    "canceled": "failed",
                },
            )
        else:
            self.add_state(
                "REACH_ARM",
                PlayMotion(motion_name="reach_arm_horizontal_gripper"),
                transitions={
                    "succeeded": "SAY_GRAB",
                    "aborted": "failed",
                    "canceled": "failed",
                },
            )

        if self.object_name is not None:
            self.add_state(
                "SAY_GRAB",
                Say(
                    text=f"Please grab the {self.object_name} in my hand. I will wait for a few seconds before releasing it.",
                ),
                transitions={
                    "succeeded": "WAIT_5",
                    "aborted": "failed",
                    "canceled": "failed",
                },
            )
        else:
            self.add_state(
                "SAY_GRAB",
                Say(
                    format_str="Please grab the {} in my hand. I will wait for a few seconds before releasing it.",
                ),
                transitions={
                    "succeeded": "WAIT_5",
                    "aborted": "failed",
                    "canceled": "failed",
                },
                remapping={"placeholders": "object_name"},
            )
        self.add_state(
            "WAIT_5",
            Wait(5),
            transitions={
                "succeeded": "OPEN_GRIPPER",
                "failed": "OPEN_GRIPPER",
            },
        )

    
        self.add_state(
            "OPEN_GRIPPER",
            PlayMotion(motion_name="open"),
            transitions={
                "succeeded": "ASK_TO_STEP_AWAY",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "ASK_TO_STEP_AWAY",
            Say(
                text="Please step back. I will put my arm away."
            ),
            transitions={
                "succeeded": "HOME",
                "aborted": "HOME",
                "canceled": "HOME",
            },
        )

        self.add_state(
            "HOME",
            PlayMotion(motion_name="home"),
            transitions={
                "succeeded": "CLOSE_GRIPPER",
                "aborted": "failed",
                "canceled": "failed",
            },
        )
        self.add_state(
            "CLOSE_GRIPPER",
            PlayMotion(motion_name="close"),
            transitions={
                "succeeded": "succeeded",
                "aborted": "failed",
                "canceled": "failed",
            },
        )


def main():

    rclpy.init()

    yasmin_ros.set_ros_loggers()

    try:
        sm = HandoverObject(object_name="bag", can_hold=True)
        bb = Blackboard()

        outcome = sm(bb)

        yasmin.YASMIN_LOG_INFO(outcome)
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)

    # Shutdown ROS 2 if it's running
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
