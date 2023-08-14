#!/usr/bin/env python3
import rospy
from coffee_shop.phases import Phase1, Phase2, Phase3
from coffee_shop.state_machine import CoffeeShop
from tiago_controllers import BaseController, HeadController
from lasr_voice import Voice
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

if __name__ == "__main__":
    rospy.init_node("coffee_shop")
    play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
    play_motion_client.wait_for_server(rospy.Duration(15.0))
    pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
    play_motion_client.send_goal_and_wait(pm_goal)
    coffee_shop = CoffeeShop(BaseController(), HeadController(), Voice())
    outcome = coffee_shop.execute()
    rospy.spin()
