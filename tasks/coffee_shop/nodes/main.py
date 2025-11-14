#!/usr/bin/env python3
import rospy
from coffee_shop.state_machine import CoffeeShop
from coffee_shop.context import Context
import sys

if __name__ == "__main__":
    rospy.init_node("coffee_shop")

    if len(sys.argv) < 3:
        rospy.signal_shutdown("No configuration was provided")
        sys.exit()
    context = Context(sys.argv[1], sys.argv[2])

    coffee_shop = CoffeeShop(context)
    outcome = coffee_shop.execute()
    context.say("I am done.")
    rospy.spin()
