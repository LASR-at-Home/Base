#!/usr/bin/env python3
# coding=utf-8
import rospy
import smach
import sys
from smach import Concurrence
from smach_ros import IntrospectionServer
from explore_surroundings_state import ExploreSurroundingsState
from look_around_state import LookAroundState
from tiago_controllers.controllers import Controllers
import smach_ros
from std_msgs.msg import Empty


def out_cb(outcome_map):
    if outcome_map['LOOK_AROUND'] == 'finished_with_known_ppl':
        return 'finished_wandering'
    if outcome_map['LOOK_AROUND'] == 'finished_with_unknown_ppl':
        return 'finished_wandering'
    else:
        return 'wander_around'

def child_term_cb(outcome_map):
    if outcome_map['LOOK_AROUND'] == 'finished_with_known_ppl':
        return True
    elif outcome_map['LOOK_AROUND'] == 'finished_with_unknown_ppl':
        return True
    elif outcome_map['MONITORING'] == 'invalid':
        return True
    else:
        return False
def monitoring_cb(ud, msg):
    return False
class MeetAndGreetSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['finished_meet_and_greet', 'aborted'])
        self.controllers = Controllers()

        with self:
            cc = Concurrence(outcomes=['finished_wandering','wander_around'],
                             default_outcome='wander_around',
                             output_keys=['detection_map'],
                             outcome_cb=out_cb,
                             child_termination_cb=child_term_cb
                             )
            with cc:
                smach.Concurrence.add('EXPLORE_SURROUNDINGS', ExploreSurroundingsState(self.controllers))
                smach.Concurrence.add('LOOK_AROUND', LookAroundState(self.controllers))
                smach.Concurrence.add('MONITORING', smach_ros.MonitorState("/sm_reset", Empty, monitoring_cb))

            smach.StateMachine.add('WANDERING', cc,
                                   transitions={'finished_wandering': 'finished_meet_and_greet', 'wander_around':'WANDERING'})



if __name__ == "__main__":
    rospy.init_node("meet_and_greet", sys.argv, anonymous=True)
    sm = MeetAndGreetSM()
    sis = IntrospectionServer('iserver', sm, 'SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()
    rospy.loginfo(f'I have completed execution with outcome {outcome}')
