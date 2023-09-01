#!/usr/bin/env python3
import smach
from coffee_shop.phases.phase_1.states import CheckTable
import rospy
from tiago_controllers import HeadController
from lasr_voice import Voice
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from lasr_object_detection_yolo.srv import YoloDetection
from coffee_shop.srv import TfTransform, TfTransformRequest

if __name__ == "__main__":
    rospy.init_node("test_check_table")
    sm = smach.StateMachine(outcomes=['end', 'not_finished'])
    play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
    play_motion_client.wait_for_server(rospy.Duration(15.0))
    rospy.wait_for_service("/yolov8/detect", rospy.Duration(15.0))
    yolo = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
    rospy.wait_for_service("/tf_transform", rospy.Duration(15.0))
    tf = rospy.ServiceProxy("/tf_transform", TfTransform)

    with sm:
        sm.add('CHECK_TABLE', CheckTable(HeadController(), Voice(), yolo, tf, play_motion_client), transitions={'finished' : 'end'})
    sm.execute()
    rospy.signal_shutdown("down")