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
from lift.defaults import DEBUG

class NavigateInLift(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['success'])

        self.default = default

    def safe_clusters_info(self, analytics, w, M):
        centers, num_clusters, midpoints, _ = analytics
        global_centers = w.local_to_global_points(M=M, points=centers, is_lift=False)
        global_centers = np.array(global_centers)

        rospy.set_param("/lift/num_clusters", num_clusters)
        rospy.set_param("/lift/centers", global_centers.tolist())

        if DEBUG > 3:
            print("num clusters in safe")
            print(rospy.get_param("/lift/num_clusters"))
            print(global_centers)
            print(type(global_centers))
            print("centers in safe")
            print(rospy.get_param("/lift/centers"))


    def execute(self, userdata):
        w = Waypoint()

        clear_costmap()
        is_from_schedule = rospy.get_param("/from_schedule")

        if not is_from_schedule:
            status = self.default.controllers.base_controller.ensure_sync_to_pose(get_pose_from_param('/wait_in_front_lift_centre/pose'))

            # get the lift information
            warped, analytics, M = w.get_lift_information(is_lift=True, is_sim=True)

            self.safe_clusters_info(analytics, w, M)

            if analytics[1] == 0:
                # if the lift is empty
                # go to predetermined place
                state = self.default.controllers.base_controller.ensure_sync_to_pose(get_pose_from_param('/lift_centre/pose'))
                return 'success'

            # get the narrow space navigation service
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
            self.default.controllers.base_controller.ensure_sync_to_pose(p)

            self.default.voice.speak("I have arrived at the lift.")

        return 'success'
