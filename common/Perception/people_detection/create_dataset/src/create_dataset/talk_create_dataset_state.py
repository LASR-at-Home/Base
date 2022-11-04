#!/usr/bin/env python3
# coding=utf-8
import rospy
import smach
import random
# from models import Person

SLEEP_TIME = 5

class Person:
    """ A person class that saves all the information about a person that the robot meets """

    def __init__(self, name, fav_drink):
        self.name = name
        self.fav_drink = fav_drink


    def get_fav_drink(self):
        return self.fav_drink

    def __str__(self):
        return (
            f"name : {self.name}\n"
            f"fav drink : {self.fav_drink}\n"
        )
class TalkCreateDatasetState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished_collecting'], output_keys=['current_person'])
        print("start")
        self.random_names = ['belly', 'mimi', 'teo', 'pipi']
        self.random_drinks = ['vodka', 'cola', 'pepsi']

    def execute(self, userdata):

        name = random.choice(self.random_names)
        print("What is your name?",name)

        rospy.sleep(SLEEP_TIME)

        drink = random.choice(self.random_drinks)
        print("What's your favourite drink?", drink)

        rospy.sleep(SLEEP_TIME)

        print(f"that's boring, i promise you will like {drink}")
        userdata.current_person = Person(name, drink)
        # userdata.current_guest = Guest(name, drink)
        return 'finished_collecting'
        # self.dialog_client.send_goal_and_wait(DialogGoal('receptionist'))
        # name = rospy.get_param("/guest/name", "unknown")
        # if name == "unknown":
        # name = None
        # # drink = rospy.get_param("/guest/drink", "unknown")
        # # if drink == "unknown":
        # drink = None