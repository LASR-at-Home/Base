#!/usr/bin/env python3
import smach
from geometry_msgs.msg import Pose, Point, Quaternion
from narrow_space_navigation.waypoints import *
from tiago_controllers.controllers.controllers import Controllers
from narrow_space_navigation.narrow_space_nav_srv import NarrowSpaceNavSrv
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from tiago_controllers.helpers.nav_map_helpers import clear_costmap
import numpy as np
import rospy

class NavigateInLift(smach.State):
    def __init__(self, controllers, voice):
        smach.State.__init__(self, outcomes=['success'])

        self.controllers = controllers
        self.voice = voice

    def execute(self, userdata):
        w = Waypoint()
        clear_costmap()
        result = self.controllers.base_controller.sync_to_pose(get_pose_from_param('/lift_in_front/pose'))
        rospy.sleep(1)
        warped, analytics, M = w.get_lift_information(is_lift=True, is_sim=True)
        if analytics[1] == 0:
            # if the lift is empty
            # go to predetermined place
            result = self.controllers.base_controller.sync_to_pose(get_pose_from_param('/lift_centre/pose'))
            return 'success'
        s = NarrowSpaceNavSrv()
        occupancy_array = warped
        thresh = np.mean(occupancy_array.flatten())
        occupancy_array[occupancy_array < thresh] = 0  # Black regions - Free space
        occupancy_array[occupancy_array >= thresh] = 100  # White regions - Occupied

        # the height map
        # get the min point to go to
        p = s.choose_target_point(occupancy_array)
        rospy.loginfo("The point to go to is {}".format(p))
        # get the global point
        global_points = w.local_to_global_points(M=M, points=p, is_lift=True)
        # get tiago there
        p = Pose()
        p.position.x = global_points[0][0]
        p.position.y = global_points[0][1]
        p.orientation.w = 1
        c = Controllers()
        c.base_controller.sync_to_pose(p)
        return 'success'


        # speak that you arrive at the lift


        # get into the lift


        # turn and speak
