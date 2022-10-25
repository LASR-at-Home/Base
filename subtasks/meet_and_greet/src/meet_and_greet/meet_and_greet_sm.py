#!/usr/bin/env python3
# coding=utf-8
import rospy
import smach
import sys
from smach import Concurrence
from smach_ros import IntrospectionServer
from create_dataset import CreateDatasetState, TalkCreateDatasetState
from train_dataset_model import TrainModelState


class MeetAndGreetSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['finished_meet_and_greet', 'wonder_around'])

        with self:
            cc = Concurrence(outcomes=['finished_dataset_collection', 'not_finished_dataset_collection', 'abort'],
                             default_outcome='not_finished_talking',
                             outcome_map=[{
                                 'finished_talking':
                                     {
                                         'TalkCreateDataset': 'finished_talking',
                                         'CreateDataset': 'finished_dataset'}
                             }])
            with cc:
                smach.Concurrence.add('TALK_CREATE_DATASET', TalkCreateDatasetState(), remapping={'current_person' : 'current_person'})
                smach.Concurrence.add('CREATE_DATASET', CreateDatasetState(), remapping={'dataset_path': 'dataset_path'})


            smach.StateMachine.add('CREATE_DATASET_AND_TALK', cc,
                                   transitions={
                                       'finished_dataset_collection' : 'TRAIN_MODEL',
                                       'not_finished_dataset_collection' : 'CREATE_DATASET_AND_TALK'
                                   })

            smach.StateMachine.add('TRAIN_MODEL', TrainModelState(), transitions={'finished_training' : 'FETCH_ATTRIBUTES', 'training_failed' : 'wonder'},
                                                                     remapping={'current_person' : 'current_guest', 'dataset_path' : 'dataset_path'})

if __name__ == "__main__":
    rospy.init_node("collect_and_scan_sm", sys.argv, anonymous=True)
    sm = MeetAndGreetSM()
    sis = IntrospectionServer('iserver', sm, 'SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()
    rospy.loginfo(f'I have completed execution with outcome {outcome}')