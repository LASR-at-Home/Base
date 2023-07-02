import smach
from .states import TakeOrder, MakeOrder, CheckOrder, GoToTable, DeliverOrder

class Phase2(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['done'])
