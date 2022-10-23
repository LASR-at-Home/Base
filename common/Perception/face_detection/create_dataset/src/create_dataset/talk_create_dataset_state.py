#!/usr/bin/env python3
# coding=utf-8
import rospy
import smach
import random



class TalkCreateDatasetState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished_collecting'], output_keys=['current_person'])
        print("start")
        self.random_names = ['hit cliff', 'terminator', 'rambo', 'rockie barboa', 'chuck norris', 'shakira']
        self.random_drinks = ['toilet water']  # , 'vodka martini', 'balvinegar', 'balsamic vinegar']

    def execute(self, userdata):

        print("Please begin moving your head at various angles, at the same time, I will ask you some questions.")
        rospy.sleep(1)

        print("What is your name?")

        rospy.sleep(5)

        name = random.choice(self.random_names)
        drink = random.choice(self.random_drinks)

        print("What's your favourite drink?")

        rospy.sleep(5)

        print(f"that's boring, you will drink {drink}")
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