#!/usr/bin/env python3
# coding=utf-8
import rospy
import smach
import random

SLEEP_TIME = 5

class TalkCreateDatasetState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished_collecting'], output_keys=['current_person'])
        print("start")
        self.random_names = ['shakira', 'rihanna', 'matteo', 'gerard']
        self.random_drinks = ['vodka', 'cola', 'pepsi']

    def execute(self, userdata):

        print("What is your name?")

        rospy.sleep(SLEEP_TIME)

        name = random.choice(self.random_names)
        drink = random.choice(self.random_drinks)

        print("What's your favourite drink?")

        rospy.sleep(SLEEP_TIME)

        print(f"that's boring, i promise you will like {drink}")
        userdata.current_guest = None
        # userdata.current_guest = Guest(name, drink)
        return 'finished_collecting'
        # self.dialog_client.send_goal_and_wait(DialogGoal('receptionist'))
        # name = rospy.get_param("/guest/name", "unknown")
        # if name == "unknown":
        # name = None
        # # drink = rospy.get_param("/guest/drink", "unknown")
        # # if drink == "unknown":
        # drink = None