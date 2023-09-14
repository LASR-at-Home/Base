import smach
from .states import GoToWaitLocation, LookForPerson, GoToPerson, GreetPerson, GuidePerson, Start, LookForPersonLaser, GoCloserToPerson

class Phase3(smach.StateMachine):
    def __init__(self, context):
        smach.StateMachine.__init__(self, outcomes=['done'])

        with self:
            smach.StateMachine.add('START_PHASE_3', Start(context), transitions={'done':'GO_TO_WAIT_LOCATION'})
            smach.StateMachine.add('GO_TO_WAIT_LOCATION', GoToWaitLocation(context), transitions={'done' : 'done', 'not done' : 'LOOK_FOR_PERSON_LASER'})
            smach.StateMachine.add('LOOK_FOR_PERSON_LASER', LookForPersonLaser(context), transitions={'found' : 'GO_CLOSER_TO_PERSON', 'not found' : 'LOOK_FOR_PERSON_LASER'})
            smach.StateMachine.add('GO_CLOSER_TO_PERSON', GoCloserToPerson(context), transitions={'done' : 'LOOK_FOR_PERSON'})
            smach.StateMachine.add('LOOK_FOR_PERSON', LookForPerson(context), transitions={'found' : 'GO_TO_PERSON', 'not found' : 'LOOK_FOR_PERSON'})
            smach.StateMachine.add('GO_TO_PERSON', GoToPerson(context), transitions={'done' : 'GREET_PERSON'})
            smach.StateMachine.add('GREET_PERSON', GreetPerson(context), transitions={'done' : 'GUIDE_PERSON'})
            smach.StateMachine.add('GUIDE_PERSON', GuidePerson(context), transitions={'done' : 'GO_TO_WAIT_LOCATION'})
