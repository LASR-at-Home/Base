#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from receptionist.states import Start
from receptionist.states import AskForName
from receptionist.states import AskForDrink
from receptionist import Default
from receptionist.states import LookForSeats
from receptionist.states import SpeakDescriptions
from receptionist.states.learn_face import LearnFaces
from receptionist.states.detect_faces import DetectFaces
#from receptionist.states.face_person import FacePerson
from lasr_skills import WaitForPersonInArea
from lasr_skills import DescribePeople
from lasr_skills import GoToLocation 




def get_pose_from_param(name):
    if not rospy.has_param(name):
        return None
    pose = rospy.get_param(name)
    print(pose)
    return Pose(Point(pose['position']['x'],
                    pose['position']['y'],
                    pose['position']['z']),
                Quaternion(pose['orientation']['x'],
                        pose['orientation']['y'],
                        pose['orientation']['z'],
                        pose['orientation']['w']))

def get_pose_from_param_new(name, col='location'):
    if not rospy.has_param(name):
        return None
    pose = rospy.get_param(name)
    position, orientation = pose[col]["position"], pose[col]["orientation"]
    return Pose(position=Point(**position), orientation=Quaternion(**orientation))




class Receptionist(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed','next_person','aborted','preempted'])
        self.default = Default()
      #  self.userdata.area_polygon = [[1.94, 0.15], [2.98, 0.28], [3.08, -0.68], [2.06, -0.84]]
       # self.userdata.depth_topic = "/xtion/depth_registered/points"
        self.userdata.wait_location = get_pose_from_param('/wait_position/pose')
        self.userdata.person_location = get_pose_from_param('/talk_position/pose')
        self.userdata.seat_location = get_pose_from_param('/seating_area_position/pose')
        with self:
            #Phase 1: 
           #
            smach.StateMachine.add('START', Start(self.default), transitions={'succeeded' : 'GO_TO_WAIT_FOR_PERSON'})
            smach.StateMachine.add('GO_TO_WAIT_FOR_PERSON',GoToLocation(),transitions={'aborted' : 'GO_TO_WAIT_FOR_PERSON','succeeded': 'WAIT_FOR_PERSON'},remapping={"location":"wait_location"})
            smach.StateMachine.add('WAIT_FOR_PERSON', WaitForPersonInArea([[1.94, 0.15], [2.98, 0.28], [3.08, -0.68], [2.06, -0.84]]) ,transitions={'failed' : 'failed','succeeded':'GO_TO_PERSON'})
            smach.StateMachine.add('GO_TO_PERSON', GoToLocation(),transitions={'succeeded':'succeeded'},remapping={'location':'person_location'})



            #Phase 2: 
           # smach.StateMachine.add('DETECT_FACES', DetectFaces(self.default), transitions={'recognised':'GO_TO_SEATING_AREA','unrecognised':'ASK_FOR_NAME'})
            #smach.StateMachine.add('ASK_FOR_NAME', AskForName(self.default),transitions={'failed':'ASK_FOR_NAME','succeeded':'ASK_FOR_DRINK'})
            #smach.StateMachine.add('ASK_FOR_DRINK', AskForDrink(self.default),transitions={'failed':'ASK_FOR_DRINK','succeeded':'LEARN_FACES'})
            #smach.StateMachine.add('LEARN_FACES', LearnFaces(self.default),transitions={'succeeded':'DESCRIBE_PEOPLE'})
            #smach.StateMachine.add('DESCRIBE_PEOPLE', DescribePeople(),transitions={'succeeded':'SPEAK_DESCRIPTIONS','failed':'GO_TO_SEATING_AREA'})
            #smach.StateMachine.add('SPEAK_DESCRIPTIONS', SpeakDescriptions(self.default), transitions={'failed':'GO_TO_SEATING_AREA','succeeded':'GO_TO_SEATING_AREA'})

            #Phase3:

            #smach.StateMachine.add('GO_TO_SEATING_AREA', GoToLocation(), transitions={'succeeded' : 'LOOK_FOR_SEATS'},remapping={'location':'seat_location'})            
            #smach.StateMachine.add('LOOK_FOR_SEATS', LookForSeats(self.default), transitions={'succeeded' : 'GO_TO_WAIT_FOR_PERSON','failed':'GO_TO_WAIT_FOR_PERSON'})
            #smach.StateMachine.add('FACE_PERSON',FacePerson(self.default),transitions={'failed':'GO_TO_WAIT_FOR_PERSON','success':'GO_TO_WAIT_FOR_PERSON'})

           # smach.StateMachine.add('END', End(self.default),transitions={'succeeded':'succeeded'})
            # smach.StateMachine.add('DESCRIBE_PEOPLE', DescribePeople(),transitions={'succeeded':'SPEAK_DESCRIPTIONS','failed':'DESCRIBE_PEOPLE'})
            # smach.StateMachine.add('SPEAK_DESCRIPTIONS', SpeakDescriptions(self.default), transitions={'failed':'DESCRIBE_PEOPLE','succeeded':'DESCRIBE_PEOPLE'})
    


