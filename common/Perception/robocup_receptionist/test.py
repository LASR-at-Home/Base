#!/usr/bin/env python

import rospy
import actionlib
from lasr_dialogflow.msg import DialogAction, DialogGoal

if __name__ == '__main__':
    rospy.init_node('test_dialog')
    client = actionlib.SimpleActionClient('/lasr_dialogflow/LasrDialogflow', DialogAction)
    print('waiting')
    client.wait_for_server()
    print('sending goal...')
    client.send_goal_and_wait(DialogGoal('receptionist'))
    #client.send_goal_and_wait(DialogGoal('escortCustomer'))

    print('goal sent!')

