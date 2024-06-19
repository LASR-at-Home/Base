#!/usr/bin/env python3

import rospy
import smach
from lasr_skills import LearnFace

if __name__ == "__main__":
    rospy.init_node("learn_face")
    # make segmentation instead for create dataset

    s = smach.StateMachine(outcomes=["succeeded", "failed"])
    with s:
        smach.StateMachine.add(
            "LEARN_FACE",
            LearnFace(dataset="receptionist", name="nicole", n_images=10),
            transitions={
                "succeeded": "succeeded",
                "failed": "failed",
            },
        )

    s.execute()
