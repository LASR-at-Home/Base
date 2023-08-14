import smach
from .states import TakeOrder, MakeOrder, CheckOrder, GoToTable, DeliverOrder, GoToCounter, InvalidateOrder

class Phase2(smach.StateMachine):
    def __init__(self, base_controller, head_controller, voice_controller):
        smach.StateMachine.__init__(self, outcomes=['done'])
        self.base_controller = base_controller
        self.head_controller = head_controller
        self.voice_controller = voice_controller
    
        with self:
            smach.StateMachine.add('GO_TO_TABLE', GoToTable(base_controller), transitions={'done' : 'TAKE_ORDER'})
            smach.StateMachine.add('TAKE_ORDER', TakeOrder(head_controller, voice_controller), transitions={'done' : 'GO_TO_COUNTER'})
            smach.StateMachine.add('GO_TO_COUNTER', GoToCounter(base_controller), transitions={'done' : 'MAKE_ORDER'})
            smach.StateMachine.add('MAKE_ORDER', MakeOrder(voice_controller), transitions={'done' : 'CHECK_ORDER'})
            smach.StateMachine.add('CHECK_ORDER', CheckOrder(voice_controller), transitions={'valid' : 'GO_TO_TABLE', 'invalid' : 'INVALIDATE_ORDER'})
            smach.StateMachine.add('INVALIDATE_ORDER', InvalidateOrder(voice_controller), transitions={'done' : 'CHECK_ORDER'})
            smach.StateMachine.add('GO_TO_TABLE', GoToTable(base_controller), transitions={'done' : 'DELIVER_ORDER'})
            smach.StateMachine.add('DELIVER_ORDER', DeliverOrder(base_controller, voice_controller), transitions={'done' : 'done'})
