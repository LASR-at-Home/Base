#!/usr/bin/env python3
import rospy
from coffee_shop.phases import Phase1, Phase2, Phase3
from coffee_shop.state_machine import CoffeeShop
from tiago_controllers import BaseController, HeadController

if __name__ == "__main__":
    rospy.init_node("coffee_shop")
    coffee_shop = CoffeeShop(BaseController(), HeadController())
    outcome = coffee_shop.execute()
    rospy.spin()
