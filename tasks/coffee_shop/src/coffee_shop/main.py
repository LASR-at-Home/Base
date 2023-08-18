#!/usr/bin/env python3
import rospy
from coffee_shop.phases import Phase1, Phase2, Phase3
from coffee_shop.state_machine import CoffeeShop
from tiago_controllers import BaseController, HeadController
from lasr_voice import Voice
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from lasr_object_detection_yolo.srv import YoloDetection
from coffee_shop.srv import TfTransform, TfTransformRequest

if __name__ == "__main__":
    rospy.init_node("coffee_shop")
    play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
    play_motion_client.wait_for_server(rospy.Duration(15.0))
    pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
    play_motion_client.send_goal_and_wait(pm_goal)
    rospy.wait_for_service("/yolov8/detect", rospy.Duration(15.0))
    yolo = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
    rospy.wait_for_service("/tf_transform", rospy.Duration(15.0))
    tf = rospy.ServiceProxy("/tf_transform", TfTransform)
    coffee_shop = CoffeeShop(BaseController(), HeadController(), Voice(), yolo, tf, play_motion_client)
    outcome = coffee_shop.execute()
    rospy.spin()
