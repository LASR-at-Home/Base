#!/usr/bin/env python3
import rospy
from coffee_shop.state_machine import CoffeeShop
from coffee_shop.context import Context
from tiago_controllers import BaseController, HeadController
from lasr_voice import Voice
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from lasr_object_detection_yolo.srv import YoloDetection
from lasr_speech.srv import Speech
from coffee_shop.srv import TfTransform, TfTransformRequest
import sys

if __name__ == "__main__":
    rospy.init_node("coffee_shop")

    if len(sys.argv) < 2:
        rospy.signal_shutdown("No configuration was provided")
        sys.exit()
    context = Context(sys.argv[1])

    coffee_shop = CoffeeShop(context)
    outcome = coffee_shop.execute()
    context.voice_controller.sync_tts("I am done.")
    rospy.spin()
