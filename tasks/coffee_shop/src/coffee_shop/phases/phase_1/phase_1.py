import smach

from .states import GoToTable, CheckTable

class Phase1(smach.StateMachine):
    def __init__(self, base_controller, head_controller, voice_controller, yolo, tf, pm):
        smach.StateMachine.__init__(self, outcomes=['done'])

        with self:
            smach.StateMachine.add('GO_TO_TABLE', GoToTable(base_controller), transitions={'done' : 'CHECK_TABLE'})
            smach.StateMachine.add('CHECK_TABLE', CheckTable(head_controller, voice_controller, yolo, tf, pm), transitions={'not_finished' : 'GO_TO_TABLE', 'finished' : 'done'})
