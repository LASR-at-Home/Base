#!/usr/bin/env python3

import rospy
import sys
import smach
from smach_ros import IntrospectionServer
from robocup_receptionist.models.guest import Guest
from smach import Concurrence
from robocup_receptionist.states.create_dataset_state import CreateDatasetState
from robocup_receptionist.states.train_model_state import TrainModelState

from hri.person_attributes.person_attributes_sm import PersonAttributesSM

import actionlib
from dialogflow_speech.msg import DialogAction, DialogGoal
from dialogflow_speech.utils import talk

# TODO: Address the faulty behaviour of CreateDatasetand if you screenshot rviz, wState, 
# TODO:                 not_enough_photos_taken -> CREATE_DATASET 
# TODO:                 if it can't train the model becuase to few dataset -> abort
# '''Should get the name and the drink of the person'''


class TalkState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished_collecting'], output_keys=['current_guest'])
        print("start")
        # self.dialog_client = actionlib.SimpleActionClient('/dialogflow_speech/dialogflow_speech', DialogAction)   
        # self.dialog_client.wait_for_server()
        self.random_names = ['hit cliff', 'terminator', 'rambo', 'rockie barboa', 'chuck norris', 'shakira']
        # self.random_names = ['amelia', 'angel', 'angel', 'charlie', 'ava', 'hunter', 'charlie', 'jack', 'charlotte', 'max', 'noah', 'hunter', 'max', 'oliver', 'mia', 'parker', 'olivia', 'sam', 'parker', 'thomas', 'sam', 'william']
        self.random_drinks = ['toilet water'] #, 'vodka martini', 'balvinegar', 'balsamic vinegar']
        # self.random_drinks = ['latte', 'coffee', 'matcha', 'tea', 'water', 'milk', 'juice', 'soda', 'beer', 'wine', 'whisky', 'vodka', 'rum', 'gin', 'tequila', 'brandy']

    def execute(self, userdata):

        talk("Please begin moving your head at various angles, at the same time, I will ask you some questions.")
        rospy.sleep(1)
        
        talk("What is your name?")

        rospy.sleep(5)

        name = random.choice(self.random_names)
        drink = random.choice(self.random_drinks)

        talk(f"Okay, I will give you a nickname, I willcall you {name}")

        talk("What's your favourite drink?")

        rospy.sleep(5)

        talk(f"that's boring, you will drink {drink}")
        userdata.current_guest = Guest(name, drink)
        return 'finished_collecting'
        # self.dialog_client.send_goal_and_wait(DialogGoal('receptionist'))
        # name = rospy.get_param("/guest/name", "unknown")
        # if name == "unknown":
        # name = None
        # # drink = rospy.get_param("/guest/drink", "unknown")
        # # if drink == "unknown":
        # drink = None

        if not name:
            name = random.choice(self.random_names)
            talk(f"Whatever, I'll just call you {name}")

        talk("What's your favourite drink?")

        rospy.sleep(5)

        if not drink:
            drink = random.choice(self.random_drinks)
            talk(f"I bet that tastes like piss, I think you should drink {drink}.")

        userdata.current_guest = Guest(name, drink)
        return 'finished_collecting'

'''Collects information from the guest and creates a dataset with information about the person'''
class CollectInfoAndScanSM(smach.StateMachine):
    def __init__(self, base_controller):
        smach.StateMachine.__init__(self, outcomes=['finished_collection_and_scan', 'wait_for_guest'], output_keys=['current_guest'])

        with self:
            cc = Concurrence(outcomes = ['finished_introduction', 'not_finished_introduction'],
                             default_outcome='not_finished_introduction',
                             output_keys=['current_guest', 'path'],
                             outcome_map = {'finished_introduction' : {'TALK_SM': 'finished_collecting',
                                                                       'SCAN_SM': 'finished_scan'}})
            with cc:
                smach.Concurrence.add('TALK_SM', TalkState(), remapping={'current_guest' : 'current_guest'})
                smach.Concurrence.add('SCAN_SM', CreateDatasetState(base_controller), remapping={'path' : 'path'})

            smach.StateMachine.add('SCAN_AND_TALK', cc,
                                                    transitions={'finished_introduction': 'TRAIN_MODEL', 'not_finished_introduction':'SCAN_AND_TALK'},
                                                    remapping={'current_guest':'current_guest'}
                            )
            smach.StateMachine.add('THANK_GUEST_FOR_MOVING_HEAD', TalkState('Thank you very much for moving your head!'), 
                                                                    transitions={'finished_conversation':'FETCH_ATTRIBUTES'} )

            smach.StateMachine.add('TRAIN_MODEL', TrainModelState(), transitions={'finished_training' : 'FETCH_ATTRIBUTES', 'training_failed' : 'wait_for_guest'}, 
                                                                     remapping={'current_guest' : 'current_guest', 'path' : 'path'})

            smach.StateMachine.add('FETCH_ATTRIBUTES', PersonAttributesSM(),
                                                        transitions={'finished_fetching': 'finished_collection_and_scan'},
                                                        remapping={'current_guest' : 'current_guest'}
            )

if __name__ == "__main__":
    rospy.init_node("collect_and_scan_sm", sys.argv)
    sm = TalkState()
    sm.execute(smach.UserData())
    rospy.spin()
    # sm = CollectInfoAndScanSM()
    # sis = IntrospectionServer('iserver', sm, 'SM_ROOT')
    # sis.start()
    # outcome = sm.execute()
    # sis.stop()
    # rospy.loginfo(f'I have completed execution with outcome {outcome}')1
