#!/usr/bin/env python3
import rospy
import rosparam

rospy.init_node("add_coffee_shop_vo")

x1, y1 = -5.8, -2.0
x2, y2 = -5.8, -0.5
x3, y3 = -4.27, -0.63
x4, y4 = -4.27, -2.18
vo_dict = {
    "vo_0096": ["submap_0", "pillar", x1, y1, 0.0],
    "vo_0097": ["submap_0", "pillar", x2, y2, 0.0],
    "vo_0098": ["submap_0", "pillar", x3, y3, 0.0],
    "vo_0099": ["submap_0", "pillar", x4, y4, 0.0],
}

for vo, params in vo_dict.items():
    rosparam.upload_params(f"mmap/vo/submap_0/{vo}", params)
