# !/usr/bin/env python3
import rospy
import os
import shutil
import smach
from .extract_embeddings import extract
from .train_model import train_model as train


class TrainModelState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['finished_training', 'training_failed'],
                             input_keys=['current_person', 'dataset_path'],  # ! comment for tesing purposes
                             output_keys=['current_person'])  # ! comment for tesing purposes

    def execute(self, userdata):
        print("I'm saving your face, so I can recognise you again later. You can stop moving your head.")
        new = userdata.dataset_path.replace(userdata.dataset_path.split(os.path.sep)[-1], userdata.current_person.name)
        if os.path.exists(new):
            shutil.rmtree(new)

        os.renames(userdata.dataset_path,
                   userdata.dataset_path.replace(userdata.dataset_path.split(os.path.sep)[-1], userdata.current_person.name))

        if len(next(os.walk(os.path.dirname(userdata.dataset_path)))[1]) > 1:
            extract()
            train()
            return 'finished_training'
        else:
            rospy.logwarn("At least 2 datasets are required for training.")
            return 'training_failed'

if __name__ == '__main__':
    rospy.init_node('smach_example_state_machine')
    sm = smach.StateMachine(outcomes=['success'])
    with sm:
        smach.StateMachine.add('TRAIN_MODEL', TrainModelState(),
                               transitions={'finished_training':'success'},
                               remapping={'dataset_path':'dataset_path',
                                          'current_person':'current_person'})
    outcome = sm.execute()
    rospy.loginfo('I have completed execution with outcome: ')
    rospy.loginfo(outcome)
