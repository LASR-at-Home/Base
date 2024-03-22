import smach
from .states import AskForName
from .states import AskForDrink
from .states import LearnFaces
from .states import DescribePeople
from .states import SpeakDescriptions

class Phase1(smach.StateMachine):
    def __init__(self, default):
        smach.StateMachine.__init__(self, outcomes=['success'])

        with self:
            smach.StateMachine.add('ASK_FOR_NAME', AskForName(self.default),transitions={'failed':'ASK_FOR_NAME','succeeded':'ASK_FOR_DRINK'})
            smach.StateMachine.add('ASK_FOR_DRINK', AskForDrink(self.default),transitions={'failed':'ASK_FOR_DRINK','succeeded':'LEARN_FACES'})
            smach.StateMachine.add('LEARN_FACES', LearnFaces(self.default),transitions={'succeeded':'DESCRIBE_PEOPLE'})
            smach.StateMachine.add('DESCRIBE_PEOPLE', DescribePeople(),transitions={'succeeded':'SPEAK_DESCRIPTIONS','failed':'succeeded'})
            smach.StateMachine.add('SPEAK_DESCRIPTIONS', SpeakDescriptions(self.default), transitions={'failed':'succeeded','succeeded':'succeeded'})




            