#!/usr/bin/env python3
import rospy
import os
import shutil
import smach
from face_detection.extract_embeddings import extract
from face_detection.train_model import train
from dialogflow_speech.utils import talk


class TrainModelState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['finished_training', 'training_failed'],
                             input_keys=['current_guest', 'path'],  #! comment for tesing purposes
                             output_keys=['current_guest'])  #! comment for tesing purposes
        
    def execute(self, userdata):
        talk("I'm saving your face, so I can recognise you again later. You can stop moving your head.")
        new = userdata.path.replace(userdata.path.split(os.path.sep)[-1], userdata.current_guest.name)
        if os.path.exists(new):
            shutil.rmtree(new)

        os.renames(userdata.path, userdata.path.replace(userdata.path.split(os.path.sep)[-1], userdata.current_guest.name))
        
        if len(next(os.walk(os.path.dirname(userdata.path)))[1]) > 1:
            extract()
            train()
            return 'finished_training'
        else:
            rospy.logwarn("At least 2 datasets are required for training.")
            return 'training_failed'
