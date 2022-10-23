#!/usr/bin/env python3
# coding=utf-8
import rospy
import smach
import sys
from smach import Concurrence
from smach_ros import IntrospectionServer
from create_dataset import CreateDatasetState, TalkCreateDatasetState
from create_dataset.train_model import train_model


class MeetAndGreetSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['finished_meet_and_greet', 'wonder_around'])

        with self:
            cc = Concurrence(outcomes=['finished_talking', 'not_finished_talking', 'abort'],
                             default_outcome='not_finished_talking',
                             outcome_map=[{
                                 'finished_talking':
                                     {
                                         'TalkCreateDataset': 'finished_talking',
                                         'CreateDataset': 'finished_dataset'}
                             }])
            with cc:
                smach.Concurrence.add('TALK_CREATE_DATASET', TalkCreateDatasetState())
                smach.Concurrence.add('CREATE_DATASET', CreateDatasetState())


            smach.StateMachine.add('CREATE_DATASET_AND_TALK', cc)

if __name__ == "__main__":
    rospy.init_node("collect_and_scan_sm", sys.argv, anonymous=True)
    sm = MeetAndGreetSM()
    sis = IntrospectionServer('iserver', sm, 'SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()
    rospy.loginfo(f'I have completed execution with outcome {outcome}')