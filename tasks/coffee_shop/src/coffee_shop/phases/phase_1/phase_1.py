import smach

from .states import Start, GoToTable, CheckTable

class Phase1(smach.StateMachine):
    def __init__(self, base_controller, head_controller, voice_controller, yolo, tf, pm, shapely):
        smach.StateMachine.__init__(self, outcomes=['done'])

        with self:
            smach.StateMachine.add('START_PHASE_1', Start(voice_controller), transitions={'done' : 'GO_TO_TABLE'})
            smach.StateMachine.add('GO_TO_TABLE', GoToTable(base_controller, voice_controller), transitions={'done' : 'CHECK_TABLE'})
            smach.StateMachine.add('CHECK_TABLE', CheckTable(head_controller, voice_controller, yolo, tf, pm, shapely), transitions={'not_finished' : 'GO_TO_TABLE', 'finished' : 'done'})
