#!/usr/bin/env python3
import rospy
import rosservice
import smach

from std_srvs.srv import Empty
from dialogflow_speech.utils import talk

class PointToPerson(smach.State):

    def __init__(self, base_controller):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'finished_introductions', 'already_introduced'],
            input_keys=["introduction_combinations"],
            output_keys=["guest1", "guest2"]
        )
        self.base_controller = base_controller

        if "point" in rosservice.get_service_list():
            rospy.wait_for_service("point")
            self.point = rospy.ServiceProxy("point", Empty)
        else:
            self.point = lambda : None
        if rospy.get_published_topics(namespace='/move_base'):
            self.face = self.base_controller.sync_face_to
        else:
            self.face = lambda x : None
            
    def execute(self, userdata):
        if not userdata.introduction_combinations:
            return 'finished_introductions'
        
        guest1, guest2 = userdata.introduction_combinations.pop(0)

        if guest2.name in guest1.introduced_to:
            return 'already_introduced'

        self.face(guest1.last_known_pose.point.x, guest1.last_known_pose.point.y)
        self.point()

        userdata.guest1, userdata.guest2 = guest1, guest2

        return 'succeeded'

class LookAtPerson(smach.State):

    def __init__(self, head_controller):
        smach.State.__init__(
            self,
            outcomes=['succeeded'],
            input_keys=["guest"]
        )
        self.head_controller = head_controller

        if self.head_controller:
            self.look = self.head_controller.sync_look_to_point
        else:
            self.look = lambda p : None
   
    def execute(self, userdata):
        
        rospy.loginfo(f"Looking at guest {userdata.guest.name}")

        self.look(userdata.guest.last_known_pose)

        return 'succeeded'


class SayIntroduction(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded'],
            input_keys=["guest1", "guest2"]
        )
        
    def execute(self, userdata):

        rospy.loginfo(f"Describing guest {userdata.guest1.name} to {userdata.guest2.name}")

        talk(f"Hello, {userdata.guest2.name}")
        talk(f"This is {userdata.guest1.name}")
        if userdata.guest1.fav_drink:
            talk(f"Their favourite drink is {userdata.guest1.fav_drink}")
        if userdata.guest1.age_range != (0,0):
            talk(f"They are around {userdata.guest1.age_range[0]} to {userdata.guest1.age_range[1]} years old.")
        if userdata.guest1.gender:
            talk(f"Their gender is {userdata.guest1.gender}, but they may identify otherwise.")
        if userdata.guest1.hair_colour:
            talk(f"The colour of their hair is {userdata.guest1.hair_colour}")
        if userdata.guest1.tshirt_colour:
            talk(f"They are wearing a {userdata.guest1.tshirt_colour} t-shirt.")

        userdata.guest1.introduced_to.append(userdata.guest2.name)

        return 'succeeded'

class Tuck(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        if "tuck" in rosservice.get_service_list():
            rospy.wait_for_service("tuck")
            self.tuck = rospy.ServiceProxy("tuck", Empty)
        else:
            self.tuck = lambda : None

    def execute(self, userdata):
        self.tuck()
        return 'succeeded'

class IntroduceGuestSM(smach.StateMachine):

    def __init__(self, base_controller, head_controller):
        smach.StateMachine.__init__(self, outcomes=['succeeded'], input_keys=['introduction_combinations'], output_keys=['introduction_combinations'])

        with self:

            smach.StateMachine.add(
                'POINT_TO_GUEST',
                PointToPerson(base_controller),
                transitions = {'succeeded' : 'LOOK_TO_GUEST', 'finished_introductions':'succeeded', 'already_introduced' : 'POINT_TO_GUEST'},
                remapping={
                    'introduction_combinations' : 'introduction_combinations',
                    'guest1' : 'guest1', 'guest2' : 'guest2'
                }
            )

            smach.StateMachine.add(
                'LOOK_TO_GUEST',
                LookAtPerson(head_controller),
                transitions = {'succeeded' : 'INTRODUCE_GUEST'},
                remapping= {'guest' : 'guest2'}
            )

            smach.StateMachine.add(
                'INTRODUCE_GUEST',
                SayIntroduction(),
                transitions = {'succeeded' : 'TUCK'},
                remapping= {'guest1' : 'guest1', 'guest2' : 'guest2'}
            )

            smach.StateMachine.add(
                'TUCK',
                Tuck(),
                transitions = {'succeeded' : 'POINT_TO_GUEST'}
            )
