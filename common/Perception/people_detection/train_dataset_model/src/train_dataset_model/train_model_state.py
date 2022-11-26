#!/usr/bin/env python3
import rospy
import os, rospkg
import shutil
import smach
from extract_embeddings import extract
from train_model import train_model as train


class TrainModelState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['finished_training', 'training_failed'],
                             input_keys=['current_person', 'dataset_path'],  # ! comment for tesing purposes
                             output_keys=['current_person'])  # ! comment for tesing purposes


    def execute(self, userdata):
        name = 'matteo'
        dataset_path = os.path.join(rospkg.RosPack().get_path('create_dataset'),  'dataset', 'mateo')
        new = dataset_path.replace(dataset_path.split(os.path.sep)[-1], name)
        if os.path.exists(new):
            shutil.rmtree(new)

        os.renames(dataset_path,
                   dataset_path.replace(dataset_path.split(os.path.sep)[-1], name))

        if len(next(os.walk(os.path.dirname(dataset_path)))[1]) > 1:
            extract()
            train()
            return 'finished_training'
        else:
            rospy.logwarn("At least 2 datasets are required for training.")
            return 'training_failed'

if __name__ == '__main__':
    rospy.init_node('smach_example_state_machine')
    name = 'alice'
    dataset_path = os.path.join(rospkg.RosPack().get_path('create_dataset'),  'dataset', 'alice')
    sm = smach.StateMachine(outcomes=['success', 'failed'])
    with sm:
        smach.StateMachine.add('TRAIN_MODEL', TrainModelState(),
                               transitions={'finished_training':'success', 'training_failed':'failed'},
                               remapping={'dataset_path':'dataset_path',
                                          'current_person':'current_person'})
    outcome = sm.execute()
    rospy.loginfo('I have completed execution with outcome: ')
    rospy.loginfo(outcome)
