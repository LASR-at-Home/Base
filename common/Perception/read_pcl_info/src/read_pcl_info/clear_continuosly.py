#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty

class ClearCostmapNode:
    def __init__(self):
        rospy.init_node('clear_costmap_node')
        self.clear_costmap_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        self.clear_interval = rospy.Duration(30.0)  # Adjust the clear interval as needed
        self.last_clear_time = rospy.Time.now()

    def run(self):
        rate = rospy.Rate(1)  # Run at 1 Hz
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            if (current_time - self.last_clear_time) >= self.clear_interval:
                self.clear_costmap()
                self.last_clear_time = current_time
            rate.sleep()

    def clear_costmap(self):
        try:
            self.clear_costmap_service.call()
            rospy.loginfo("Costmap cleared.")
        except rospy.ServiceException as e:
            rospy.logerr("Failed to clear costmap: {}".format(e))

if __name__ == '__main__':
    try:
        clear_costmap_node = ClearCostmapNode()
        clear_costmap_node.run()
    except rospy.ROSInterruptException:
        pass
