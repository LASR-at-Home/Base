#!/usr/bin/env python3
import smach
import rospy
import googleapiclient
# from dialogflow.receptionist_conversation_API import ReceptionistAPI
from google.api_core.exceptions import DeadlineExceeded

class AwaitConvo(smach.State):
    def __init__(self, api):
        smach.State.__init__(self, 
                            outcomes=['heard_person', 'person_not_heard'],
                            output_keys=['action_goal'], 
                            )
        self.api = api

    def execute(self, ud):
        try:
            self.api.start_conversation()
            if self.api.name == None:
                ud.action_goal = 'getName'
                return 'heard_person'
            elif self.api.favouriteDrink == None:
                ud.action_goal = 'getFavouriteDrink'
                return 'heard_person'
            else:
                ud.action_goal = 'goodbye'
                return 'heard_person'
        except rospy.exceptions.ROSException:#, googleapiclient.exception.DeadlineExceeded:
            return 'person_not_heard'
        except DeadlineExceeded:#, googleapiclient.exception.DeadlineExceeded:
            return 'person_not_heard'


class ActionState(smach.State):
    def __init__(self, api):
        smach.State.__init__(self,  
                            outcomes =['success', 'failure'],
                            input_keys=['action_goal'],
                            output_keys=['name', 'drink']
                            )
        self.api = api

    def execute(self, ud):
        try:
            if ud.action_goal == 'getName':
                ud.name = self.api.get_name()
                ud.drink = None
            elif ud.action_goal == 'getFavouriteDrink':
                ud.drink = self.api.get_favourite_drink()
                ud.name = None
            else:
                pass
            return 'success'
        except rospy.exceptions.ROSException:
            return 'failure'
        except DeadlineExceeded:
            return 'failure'

class EndConvo(smach.State):
    def __init__(self, api):
        smach.State.__init__(self, 
                            outcomes=['name_known, drink_known', 'name_unknown, drink_known','name_known, drink_unknown','name_unknown, drink_unknown'], #4 different outcomes
                            input_keys=['name','drink'],
                            output_keys=['action_goal'],
                            )
        self.api = api
        self.name = None
        self.favourite_drink = None

    def execute(self, ud):
        if(ud.name):
            self.name = ud.name
        
        if(ud.drink):
            self.favourite_drink = ud.drink
        
        if self.name and self.favourite_drink:
            return 'name_known, drink_known'
        elif self.name:
            ud.action_goal ='getFavouriteDrink'
            return 'name_known, drink_unknown'
        elif self.favourite_drink:
            ud.action_goal = 'getName'
            return 'name_unknown, drink_known'
        else:
            ud.action_goal = 'getName'
            return 'name_unknown, drink_unknown'

class Talk(smach.StateMachine):
    def __init__(self):
        self.api = ReceptionistAPI()
        smach.StateMachine.__init__(self, 
        outcomes=['finished_collecting','still_in_conversation','preempted'],
        )

        with self:
            smach.StateMachine.add(
                'Await Person', 
                AwaitConvo(self.api),
                transitions={
                    'heard_person' : 'Ask Person',
                    'person_not_heard' : 'Await Person',
                }
            )
            smach.StateMachine.add(
                'Ask Person', 
                ActionState(self.api),
                transitions={
                    'success' : 'End Convo',
                    'failure' : 'Ask Person',
                }
            )
            smach.StateMachine.add(
                'End Convo', 
                EndConvo(self.api),
                transitions={
                    'name_unknown, drink_unknown': 'Ask Person',
                    'name_unknown, drink_known': 'Ask Person',
                    'name_known, drink_unknown': 'Ask Person',
                    'name_known, drink_known': 'finished_collecting',
                }
            )

if __name__ == "__main__":
    rospy.init_node("talk_sm")
    sm = Talk()
    # sis = IntrospectionServer('iserver', sm, 'SM_ROOT')
    # sis.start()
    outcome = sm.execute()
    # sis.stop()
    rospy.loginfo('I have completed execution with outcome: name : {name}, drink : {drink}'.format(
        name=sm.api.name, drink = sm.api.favourite_drink
    )
    
    )


