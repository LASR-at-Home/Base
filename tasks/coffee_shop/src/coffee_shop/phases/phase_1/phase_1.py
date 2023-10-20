import smach

from .states import Start, GoToTable, CheckTable, GoToPreTable, PreCheckTable

class Phase1(smach.StateMachine):
    def __init__(self, context):
        smach.StateMachine.__init__(self, outcomes=['done'])

        with self:
            smach.StateMachine.add('START_PHASE_1', Start(context), transitions={'done' : 'GO_TO_TABLE'})
           # smach.StateMachine.add('GO_TO_TABLE', GoToPreTable(context), transitions={'done' : 'GO_TO_TABLE'})
            #smach.StateMachine.add('PRE_CHECK_TABLE', PreCheckTable(context), transitions={'done' : 'GO_TO_TABLE'})
            smach.StateMachine.add('GO_TO_TABLE', GoToTable(context), transitions={'done' : 'CHECK_TABLE'})
            smach.StateMachine.add('CHECK_TABLE', CheckTable(context), transitions={'not_finished' : 'GO_TO_TABLE', 'finished' : 'done'})
