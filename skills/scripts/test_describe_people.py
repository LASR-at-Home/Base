#!/usr/bin/env python3

## 
## IGNORE THIS FILE IT WILL BE DELETED ON MERGE
##                   - paul
## 

import smach
import rospy

from lasr_skills import TestDescribePeople

if __name__ == "__main__":
    rospy.init_node("test_describe")

    sm = smach.StateMachine(outcomes=['end'], output_keys=['people'])

    with sm:
        sm.add('DESCRIBE', TestDescribePeople(), transitions={'succeeded' : 'end', 'failed': 'end'})

    sm.execute()

    print('\n\nDetected people:', sm.userdata['people'])

    rospy.signal_shutdown("down")
