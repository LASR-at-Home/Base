import smach

from .states import GoToTable, CheckTable

class Phase1(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=[])

        with self:
            smach.StateMachine.add('CHECK_TABLE', CheckTable())
