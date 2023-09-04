#!/usr/bin/env python3
import rospy
from coffee_shop.phases import Phase1, Phase2, Phase3
from coffee_shop.state_machine import CoffeeShop
from tiago_controllers import BaseController, HeadController
from lasr_voice import Voice
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from lasr_object_detection_yolo.srv import YoloDetection
from lasr_speech.srv import Speech
from coffee_shop.srv import TfTransform, TfTransformRequest
from lasr_shapely import LasrShapely

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
    voice = Voice()
    shapely = LasrShapely()
    rospy.wait_for_service("/lasr_speech/transcribe_and_parse")
    speech = rospy.ServiceProxy("/lasr_speech/transcribe_and_parse", Speech)
    coffee_shop = CoffeeShop(BaseController(), HeadController(), voice, yolo, tf, play_motion_client, speech, shapely)
    outcome = coffee_shop.execute()
    voice.sync_tts("I am done.")
    rospy.spin()
