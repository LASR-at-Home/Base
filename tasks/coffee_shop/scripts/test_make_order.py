#!/usr/bin/env python3
import smach
from coffee_shop.phases.phase_2.states import MakeOrder, CheckOrder, InvalidateOrder
import rospy
from lasr_voice import Voice
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from lasr_object_detection_yolo.srv import YoloDetection
from coffee_shop.srv import TfTransform, TfTransformRequest

if __name__ == "__main__":
    rospy.init_node("test_check_table")
    sm = smach.StateMachine(outcomes=['end'])
    voice_controller = Voice()
    play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
    play_motion_client.wait_for_server(rospy.Duration(15.0))
    rospy.wait_for_service("/yolov8/detect", rospy.Duration(15.0))
    yolo = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
    rospy.wait_for_service("/tf_transform", rospy.Duration(15.0))
    tf = rospy.ServiceProxy("/tf_transform", TfTransform)

    with sm:
        sm.add('MAKE_ORDER', MakeOrder(voice_controller), transitions={'done' : 'CHECK_ORDER'})
        sm.add('CHECK_ORDER', CheckOrder(voice_controller, yolo, tf, play_motion_client), transitions={'correct': 'end', 'incorrect': 'INVALIDATE_ORDER'})
        smach.StateMachine.add('INVALIDATE_ORDER', InvalidateOrder(voice_controller), transitions={'done': 'CHECK_ORDER'})
    sm.execute()
    rospy.signal_shutdown("down")
