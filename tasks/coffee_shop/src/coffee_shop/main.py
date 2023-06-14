#!/usr/bin/env python3
import rospy
from coffee_shop.phases import Phase1, Phase2, Phase3
from coffee_shop.state_machine import CoffeeShop

if __name__ == "__main__":
    coffee_shop = CoffeeShop()
    rospy.init_node("coffee_shop")
    outcome = coffee_shop.execute()
    rospy.spin()
