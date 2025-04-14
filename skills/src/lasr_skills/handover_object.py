#!/usr/bin/env python3
import smach
import smach_ros

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

from lasr_skills import Say, PlayMotion, Wait

from ament_index_python.packages import get_package_share_directory
import yaml
import os

from typing import Union


class HandoverObject(smach.StateMachine):
    def __init__(
        self, node: Node, object_name: Union[str, None] = None, vertical: bool = True
    ):

        if object_name is not None:
            super(HandoverObject, self).__init__(outcomes=["succeeded", "failed"])
        else:
            smach.StateMachine.__init__(
                self, outcomes=["succeeded", "failed"], input_keys=["object_name"]
            )
        self.node = node
        self.load_motion_params()

        with self:
            smach.StateMachine.add(
                "CLEAR_OCTOMAP",
                smach_ros.ServiceState("clear_octomap", Empty),
                transitions={
                    "succeeded": "LOOK_LEFT",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "LOOK_LEFT",
                PlayMotion(node=Node, motion_name="look_left"),
                transitions={
                    "succeeded": "LOOK_DOWN_LEFT",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "LOOK_DOWN_LEFT",
                PlayMotion(node=Node, motion_name="look_down_left"),
                transitions={
                    "succeeded": "LOOK_RIGHT",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "LOOK_RIGHT",
                PlayMotion(node=Node, motion_name="look_right"),
                transitions={
                    "succeeded": "LOOK_DOWN_RIGHT",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "LOOK_DOWN_RIGHT",
                PlayMotion(node=Node, motion_name="look_down_right"),
                transitions={
                    "succeeded": "LOOK_DOWN_CENTRE",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )
            # TODO: check whether the motion name for state LOOK_DOWN_CENTRE in ROS1 was look_centre is not look_down_centre on purpose and not just a mistake
            smach.StateMachine.add(
                "LOOK_DOWN_CENTRE",
                PlayMotion(node=Node, motion_name="look_centre"),
                transitions={
                    "succeeded": "LOOK_CENTRE",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "LOOK_CENTRE",
                PlayMotion(node=Node, motion_name="look_centre"),
                transitions={
                    "succeeded": "SAY_REACH_ARM",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_REACH_ARM",
                Say(
                    node=Node, text="Please step back, I am going to reach my arm out."
                ),
                transitions={
                    "succeeded": "REACH_ARM",
                    "aborted": "REACH_ARM",
                    "preempted": "REACH_ARM",
                },
            )
            if vertical:
                smach.StateMachine.add(
                    "REACH_ARM",
                    PlayMotion(node=Node, motion_name="reach_arm_vertical_gripper"),
                    transitions={
                        "succeeded": "SAY_TAKE",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )
            else:
                smach.StateMachine.add(
                    "REACH_ARM",
                    PlayMotion(node=Node, motion_name="reach_arm_horizontal_gripper"),
                    transitions={
                        "succeeded": "SAY_TAKE",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )

            if object_name is not None:
                smach.StateMachine.add(
                    "SAY_TAKE",
                    Say(
                        node=Node,
                        text=f"Please grab the {object_name} in my hand. I will wait for a few seconds.",
                    ),
                    transitions={
                        "succeeded": "OPEN_GRIPPER",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )
            else:
                smach.StateMachine.add(
                    "SAY_TAKE",
                    Say(
                        node=Node,
                        format_str="Please take the {} from my hand. I will wait for a few seconds.",
                    ),
                    transitions={
                        "succeeded": "OPEN_GRIPPER",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                    remapping={"placeholders": "object_name"},
                )

            smach.StateMachine.add(
                "WAIT_5",
                Wait(5),
                transitions={"succeeded": "FOLD_ARM", "failed": "OPEN_GRIPPER"},
            )

            smach.StateMachine.add(
                "OPEN_GRIPPER",
                PlayMotion(node=Node, motion_name="open_gripper"),
                transitions={
                    "succeeded": "WAIT_5",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "FOLD_ARM",
                PlayMotion(node=Node, motion_name="home"),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

    def load_motion_params(self):
        package_path = get_package_share_directory("lasr_skills")
        config_path = os.path.join(package_path, "config", "motion.yaml")
        if os.path.exists(config_path):
            with open(config_path, "r") as f:
                params = yaml.safe_load(f)
                for key, value in params.items():
                    self.node.declare_parameters(key, value)


if __name__ == "__main__":

    rclpy.init()
    node = rclpy.create_node("handover_object")
    sm = HandoverObject(node=node, object_name="cola", vertical=True)
    outcome = sm.execute()
    node.get_logger().info(f"Outcome: {outcome}")
    rclpy.shutdown()
