#!/usr/bin/env python3
import smach
import rospy
import sys

from lasr_skills import TestDescribePeople

if __name__ == "__main__":
    rospy.init_node("test_describe")

    sm = smach.StateMachine(outcomes=['end'])

    with sm:
        sm.add('DESCRIBE', TestDescribePeople(), transitions={'succeeded' : 'end', 'failed': 'end'})

    sm.execute()
    rospy.signal_shutdown("down")