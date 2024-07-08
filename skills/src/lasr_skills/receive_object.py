#!/usr/bin/env python3
import smach
import smach_ros

from std_srvs.srv import Empty

from lasr_skills import Say, ListenFor, PlayMotion

import rospkg
import rosparam
import os

from typing import Union


class ReceiveObject(smach.StateMachine):
    def __init__(self, object_name: Union[str, None] = None, vertical: bool = True):

        if object_name is not None:
            super(ReceiveObject, self).__init__(outcomes=["succeeded", "failed"])
        else:
            smach.StateMachine.__init__(
                self, outcomes=["succeeded", "failed"], input_keys=["object_name"]
            )

        r = rospkg.RosPack()
        els = rosparam.load_file(
            os.path.join(r.get_path("lasr_skills"), "config", "motions.yaml")
        )
        for param, ns in els:
            rosparam.upload_params(ns, param)

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
                PlayMotion(motion_name="look_left"),
                transitions={
                    "succeeded": "LOOK_DOWN_LEFT",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "LOOK_DOWN_LEFT",
                PlayMotion(motion_name="look_down_left"),
                transitions={
                    "succeeded": "LOOK_RIGHT",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "LOOK_RIGHT",
                PlayMotion(motion_name="look_right"),
                transitions={
                    "succeeded": "LOOK_DOWN_RIGHT",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "LOOK_DOWN_RIGHT",
                PlayMotion(motion_name="look_down_right"),
                transitions={
                    "succeeded": "LOOK_DOWN_CENTRE",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "LOOK_DOWN_CENTRE",
                PlayMotion(motion_name="look_centre"),
                transitions={
                    "succeeded": "LOOK_CENTRE",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "LOOK_CENTRE",
                PlayMotion(motion_name="look_centre"),
                transitions={
                    "succeeded": "SAY_REACH_ARM",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_REACH_ARM",
                Say(text="Please step back, I am going to reach my arm out."),
                transitions={
                    "succeeded": "REACH_ARM",
                    "aborted": "REACH_ARM",
                    "preempted": "REACH_ARM",
                },
            )

            if vertical:
                smach.StateMachine.add(
                    "REACH_ARM",
                    PlayMotion(motion_name="reach_arm_vertical_gripper"),
                    transitions={
                        "succeeded": "OPEN_GRIPPER",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )
            else:
                smach.StateMachine.add(
                    "REACH_ARM",
                    PlayMotion(motion_name="reach_arm_horizontal_gripper"),
                    transitions={
                        "succeeded": "OPEN_GRIPPER",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )

            smach.StateMachine.add(
                "OPEN_GRIPPER",
                PlayMotion(motion_name="open_gripper"),
                transitions={
                    "succeeded": "SAY_PLACE",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            if object_name is not None:
                smach.StateMachine.add(
                    "SAY_PLACE",
                    Say(
                        text=f"Please place the {object_name} in my end-effector, and say `I am done`.",
                    ),
                    transitions={
                        "succeeded": "LISTEN_DONE",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                )
            else:
                smach.StateMachine.add(
                    "SAY_PLACE",
                    Say(
                        format_str="Please place the {} in my end-effector, and say `I am done`.",
                    ),
                    transitions={
                        "succeeded": "LISTEN_DONE",
                        "aborted": "failed",
                        "preempted": "failed",
                    },
                    remapping={"placeholders": "object_name"},
                )
            smach.StateMachine.add(
                "LISTEN_DONE",
                ListenFor("done"),
                transitions={
                    "succeeded": "CLOSE_GRIPPER",
                    "not_done": "LISTEN_DONE",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )
            smach.StateMachine.add(
                "CLOSE_GRIPPER",
                smach_ros.ServiceState("parallel_gripper_controller/grasp", Empty),
                transitions={
                    "succeeded": "FOLD_ARM",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )
            smach.StateMachine.add(
                "FOLD_ARM",
                PlayMotion(motion_name="home"),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )


if __name__ == "__main__":
    import rospy

    rospy.init_node("receive_object")
    sm = ReceiveObject()
    sm.userdata.object_name = "cola"
    outcome = sm.execute()
    rospy.loginfo("Outcome: " + outcome)
    rospy.signal_shutdown("Done")
