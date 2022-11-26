#!/usr/bin/env python3
import rospy
import actionlib

from dialogflow_speech.msg import DialogAction, DialogGoal

if __name__ == "__main__":
   rospy.init_node("test_speech")
   client = actionlib.SimpleActionClient('/dialogflow_speech/dialogflow_speech', DialogAction)
   client.wait_for_server()
   
   client.send_goal_and_wait(DialogGoal('receptionist', 'start'))
   client.send_goal_and_wait(DialogGoal('receptionist', 'ask_name'))
   client.send_goal_and_wait(DialogGoal('receptionist', 'ask_favourite_drink'))
   client.send_goal_and_wait(DialogGoal('receptionist', 'finish'))
   rospy.spin()