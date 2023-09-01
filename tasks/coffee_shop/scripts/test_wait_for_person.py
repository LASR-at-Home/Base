#!/usr/bin/env python3
import smach
from coffee_shop.phases.phase_3.states import LookForPerson, GreetPerson, GoToPerson
import rospy
from lasr_voice import Voice
from tiago_controllers import BaseController
from lasr_object_detection_yolo.srv import YoloDetection
from coffee_shop.srv import TfTransform, TfTransformRequest

if __name__ == "__main__":
    rospy.init_node("test_wait_for_person")
    sm = smach.StateMachine(outcomes=['end'])
    play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
    yolo = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
    rospy.wait_for_service("/tf_transform", rospy.Duration(15.0))
    tf = rospy.ServiceProxy("/tf_transform", TfTransform)

    with sm:
        sm.add('LOOK_FOR_PERSON', LookForPerson(yolo, tf), transitions={'found' : 'GO_TO_PERSON', 'not found' : 'LOOK_FOR_PERSON'})
        sm.add('GO_TO_PERSON', GoToPerson(BaseController()), transitions={'done' : 'GREET_PERSON'})
        sm.add('GREET_PERSON', GreetPerson(Voice()), transitions={'done' : 'end'})
    sm.execute()
