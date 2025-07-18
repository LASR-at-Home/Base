#!/usr/bin/env python3
import smach
import smach_ros

from std_srvs.srv import Empty

from lasr_skills import Say, PlayMotion, Wait

import rospkg
import rosparam
import os

from typing import Union


class HandoverObject(smach.StateMachine):
    def __init__(self, object_name: Union[str, None] = None, vertical: bool = True):

        if object_name is not None:
            super(HandoverObject, self).__init__(outcomes=["succeeded", "failed"])
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

            if object_name is not None:
                smach.StateMachine.add(
                    "SAY_TAKE",
                    Say(
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
                        format_str="Please place the {} in my hand. I will wait for a few seconds.",
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
                transitions={"succeeded": "OPEN_GRIPPER", "failed": "OPEN_GRIPPER"},
            )

            smach.StateMachine.add(
                "OPEN_GRIPPER",
                PlayMotion(motion_name="open_gripper"),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )


if __name__ == "__main__":
    import rospy

    rospy.init_node("handover_object")
    sm = HandoverObject()
    sm.userdata.object_name = "cola"
    outcome = sm.execute()
    rospy.loginfo("Outcome: " + outcome)
    rospy.signal_shutdown("Done")
