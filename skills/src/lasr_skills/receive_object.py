#!/usr/bin/env python3
import rclpy

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

#TODO: Do we need to detect object or just assume that the second guest is holding a bag. 
class ReceiveObject(StateMachine):
    def __init__(self, object_name: Union[str, None] = None, vertical: bool = True):

        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)
        if object_name is None:
            self.add_input_key("object_name")

        self.add_state(
            "CLEAR_OCTOMAP",
            ClearOctomap(),
            transitions={
                "succeeded": "LOOK_LEFT",
                "aborted": "failed"
            },
        )

        self.add_state(
            "LOOK_LEFT",
            PlayMotion(motion_name="look_left"),
            transitions={
                "succeeded": "LOOK_DOWN_LEFT",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "LOOK_DOWN_LEFT",
            PlayMotion(motion_name="look_down_left"),
            transitions={
                "succeeded": "LOOK_RIGHT",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "LOOK_RIGHT",
            PlayMotion(motion_name="look_right"),
            transitions={
                "succeeded": "LOOK_DOWN_RIGHT",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "LOOK_DOWN_RIGHT",
            PlayMotion(motion_name="look_down_right"),
            transitions={
                "succeeded": "LOOK_DOWN_CENTRE",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "LOOK_DOWN_CENTRE",
            PlayMotion(motion_name="look_centre"),
            transitions={
                "succeeded": "LOOK_CENTRE",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "LOOK_CENTRE",
            PlayMotion(motion_name="look_centre"),
            transitions={
                "succeeded": "SAY_REACH_ARM",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "SAY_REACH_ARM",
            Say(text="Please step back, I am going to reach my arm out."),
            transitions={
                "succeeded": "REACH_ARM",
                "aborted": "REACH_ARM",
                "canceled": "REACH_ARM",
            },
        )

        if vertical:
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
            PlayMotion(motion_name="open_gripper"),
            transitions={
                "succeeded": "SAY_PLACE",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        if object_name is not None:
            self.add_state(
                "SAY_PLACE",
                Say(
                    text=f"Please place the {object_name} in my hand. I will wait for a few seconds.",
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
                    format_str="Please place the {} in my hand. I will wait for a few seconds.",
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

        #TODO: No longer a gripper server for this  smach_ros.ServiceState("/parallel_gripper_controller/grasp", Empty)
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
            "CLOSE_HALF_GRIPPER", # TEMPORARY REPLACEMENT - using half to not jam an item between gripper
            PlayMotion(motion_name="close_half"),
            transitions={
                "succeeded": "FOLD_ARM",
                "aborted": "failed",
                "canceled": "failed",
            },
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
        sm = ReceiveObject(object_name="bag")
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

