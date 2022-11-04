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
        # smach.StateMachine.__init__(self, outcomes=['finished_meet_and_greet', 'wonder_around'])
        smach.StateMachine.__init__(self, outcomes=['finished_meet_and_greet', 'wonder_around'], output_keys=['current_person'])

        with self:
            cc = Concurrence(outcomes=['finished_dataset_collection', 'not_finished_dataset_collection'],
                             default_outcome='not_finished_dataset_collection',
                             output_keys=['current_person', 'dataset_path'],
                             outcome_map={
                                 'finished_dataset_collection':
                                     {
                                         'TALK_CREATE_DATASET': 'finished_collecting',
                                         'CREATE_DATASET': 'finished_scan'}
                             })
            with cc:
                smach.Concurrence.add('TALK_CREATE_DATASET', TalkCreateDatasetState(), remapping={'current_person' : 'current_person'})
                smach.Concurrence.add('CREATE_DATASET', CreateDatasetState(), remapping={'dataset_path': 'dataset_path'})

            smach.StateMachine.add('EXPLORE_SURROUNDINGS', ExploreSurroundings())
            smach.StateMachine.add('CREATE_DATASET_AND_TALK', cc,
                                   transitions={
                                       'finished_dataset_collection' : 'TRAIN_MODEL',
                                       'not_finished_dataset_collection' : 'CREATE_DATASET_AND_TALK'
                                   })
        # smach.StateMachine.add('CREATE_DATASET', CreateDatasetState,
        #                        transitions={'finished_scan': 'TRAIN_MODEL'},
        #                        remapping={'dataset_path': 'dataset_path'}
        #                        )

            smach.StateMachine.add('TRAIN_MODEL', TrainModelState(), transitions={'finished_training' : 'finished_meet_and_greet', 'training_failed' : 'CREATE_DATASET_AND_TALK'},
                                                                     remapping={'current_person' : 'current_person', 'dataset_path' : 'dataset_path'})

if __name__ == "__main__":
    rospy.init_node("collect_and_scan_sm", sys.argv, anonymous=True)
    sm = MeetAndGreetSM()
    sis = IntrospectionServer('iserver', sm, 'SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()
    rospy.loginfo(f'I have completed execution with outcome {outcome}')


# BACKUP
# #!/usr/bin/env python3
# # coding=utf-8
# import rospy
# import smach
# import sys
# from smach import Concurrence
# from smach_ros import IntrospectionServer
# from create_dataset import CreateDatasetState, TalkCreateDatasetState
# from train_dataset_model import TrainModelState
#
#
# class MeetAndGreetSM(smach.StateMachine):
#     def __init__(self):
#         # smach.StateMachine.__init__(self, outcomes=['finished_meet_and_greet', 'wonder_around'])
#         smach.StateMachine.__init__(self, outcomes=['finished_meet_and_greet', 'wonder_around'], output_keys=['current_person'])
#
#         with self:
#             cc = Concurrence(outcomes=['finished_dataset_collection', 'not_finished_dataset_collection'],
#                              default_outcome='not_finished_dataset_collection',
#                              output_keys=['current_person', 'dataset_path'],
#                              outcome_map={
#                                  'finished_dataset_collection':
#                                      {
#                                          'TALK_CREATE_DATASET': 'finished_collecting',
#                                          'CREATE_DATASET': 'finished_scan'}
#                              })
#             with cc:
#                 smach.Concurrence.add('TALK_CREATE_DATASET', TalkCreateDatasetState(), remapping={'current_person' : 'current_person'})
#                 smach.Concurrence.add('CREATE_DATASET', CreateDatasetState(), remapping={'dataset_path': 'dataset_path'})
#
#
#             smach.StateMachine.add('CREATE_DATASET_AND_TALK', cc,
#                                    transitions={
#                                        'finished_dataset_collection' : 'TRAIN_MODEL',
#                                        'not_finished_dataset_collection' : 'CREATE_DATASET_AND_TALK'
#                                    })
#             # smach.StateMachine.add('CREATE_DATASET', CreateDatasetState,
#             #                        transitions={'finished_scan': 'TRAIN_MODEL'},
#             #                        remapping={'dataset_path': 'dataset_path'}
#             #                        )
#
#             smach.StateMachine.add('TRAIN_MODEL', TrainModelState(), transitions={'finished_training' : 'finished_meet_and_greet', 'training_failed' : 'CREATE_DATASET_AND_TALK'},
#                                    remapping={'current_person' : 'current_person', 'dataset_path' : 'dataset_path'})
#
# if __name__ == "__main__":
#     rospy.init_node("collect_and_scan_sm", sys.argv, anonymous=True)
#     sm = MeetAndGreetSM()
#     sis = IntrospectionServer('iserver', sm, 'SM_ROOT')
#     sis.start()
#     outcome = sm.execute()
#     sis.stop()
#     rospy.loginfo(f'I have completed execution with outcome {outcome}')
