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


# Update to take new param which dictates if the object is to be held (placed in gripper) or place in basket
class ReceiveObject(StateMachine):
    def __init__(
        self,
        object_name: Union[str, None] = None,
        vertical: bool = True,
        can_hold: bool = False,
    ):

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
                    text=f"Can you please hold the {self.object_name} in my gripper. I will wait a few seconds before closing it.",
                ),
                transitions={
                    "succeeded": "OPEN_GRIPPER",
                    "aborted": "failed",
                    "canceled": "failed",
                },
            )
        else:
            self.add_state(
                "REQUEST",
                Say(
                    format_str="Can you please hold the {} in my gripper. I will wait a few seconds before closing it.",
                ),
                transitions={
                    "succeeded": "OPEN_GRIPPER",
                    "aborted": "failed",
                    "canceled": "failed",
                },
                remapping={"placeholders": "object_name"},
            )

        self.add_state(
            "OPEN_GRIPPER",
            PlayMotion(motion_name="open"),
            transitions={
                "succeeded": "WAIT_5",
                "aborted": "failed",
                "canceled": "failed",
            },
        )
        self.add_state(
            "WAIT_5",
            Wait(5),
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
                "succeeded": "REQUEST",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "REQUEST",
            Say(text="Please step back, I am going to reach my arm out."),
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
                    "succeeded": "OPEN_GRIPPER",
                    "aborted": "failed",
                    "canceled": "failed",
                },
            )
        else:
            self.add_state(
                "REACH_ARM",
                PlayMotion(motion_name="reach_arm_horizontal_gripper"),
                transitions={
                    "succeeded": "OPEN_GRIPPER",
                    "aborted": "failed",
                    "canceled": "failed",
                },
            )

        self.add_state(
            "OPEN_GRIPPER",
            PlayMotion(motion_name="open"),
            transitions={
                "succeeded": "SAY_PLACE",
                "aborted": "failed",
                "canceled": "failed",
            },
        )
        if self.object_name is not None:
            self.add_state(
                "SAY_PLACE",
                Say(
                    text=f"I am ready to recieve the {self.object_name} in my hand. Please place the bag on my gripper. I will wait for a few seconds.",
                ),
                transitions={
                    "succeeded": "WAIT_5",
                    "aborted": "failed",
                    "canceled": "failed",
                },
            )
        else:
            self.add_state(
                "SAY_PLACE",
                Say(
                    format_str="I am ready to recieve the {} in my hand. I will wait for a few seconds. Please give me space to my left to put my arm away after.",
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
                "succeeded": "CLOSE_HALF_GRIPPER",
                "failed": "CLOSE_HALF_GRIPPER",
            },
        )

        # TODO: No longer a gripper server for this  smach_ros.ServiceState("/parallel_gripper_controller/grasp", Empty)
        # Alternatively:
        #   1. https://docs.pal-robotics.com/sdk/24.09/actions/advanced_grasping-grasp.html but verify Fruity has the action server
        #   2. /gripper_controller/incrementer service or
        #   3. /gripper_controller/ action server        - NOT AVAILABLE | use lasr_manipulation

        # self.add_state(
        #     "CLOSE_GRIPPER",
        #     smach_ros.ServiceState("parallel_gripper_controller/grasp", Empty),
        #     transitions={
        #         "succeeded": "FOLD_ARM",
        #         "aborted": "failed",
        #         "canceled": "failed",
        #     },
        # )
        self.add_state(
            "CLOSE_HALF_GRIPPER",  # TEMPORARY REPLACEMENT
            PlayMotion(motion_name="close"),
            transitions={
                "succeeded": "WARN_ARM",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "WARN_ARM",
            Say(
                text="I will put my arm away in 5 seconds. Please give me a lot of space to my left."
            ),
            transitions={
                "succeeded": "WAIT_PUT_ARM_AWAY",
                "aborted": "WAIT_PUT_ARM_AWAY",
                "canceled": "WAIT_PUT_ARM_AWAY",
            },
        )

        self.add_state(
            "WAIT_PUT_ARM_AWAY",
            Wait(5),
            transitions={"succeeded": "FOLD_ARM", "failed": "FOLD_ARM"},
        )

        self.add_state(
            "FOLD_ARM",
            PlayMotion(motion_name="cml_arm_away"),
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
        sm = ReceiveObject(object_name="bag", can_hold=True)
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
