#!/usr/bin/env python3
import smach
from geometry_msgs.msg import Pose, Point, Quaternion
from narrow_space_navigation.waypoints import *
from tiago_controllers.controllers.controllers import Controllers
from narrow_space_navigation.narrow_space_nav_srv import NarrowSpaceNavSrv
import numpy as np

class NavigateInLift(smach.State):
    def __init__(self, controllers, voice):
        smach.State.__init__(self, outcomes=['success'])

        self.controllers = controllers
        self.voice = voice

    def execute(self, userdata):
        w = Waypoint()
        warped, analytics, M = w.get_lift_information(is_lift=True)
        s = NarrowSpaceNavSrv()
        occupancy_array = warped
        thresh = np.mean(occupancy_array.flatten())
        occupancy_array[occupancy_array < thresh] = 0  # Black regions - Free space
        occupancy_array[occupancy_array >= thresh] = 100  # White regions - Occupied

        # get the min point to go to
        p = s.choose_target_point(occupancy_array)
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
