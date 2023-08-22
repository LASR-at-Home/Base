#!/usr/bin/env python3
from waypoints import *
from narrow_space_navigation.srv import HeightMap, HeightMapResponse, HeightMapRequest
from narrow_space_nav_srv import NarrowSpaceNavSrv
from tiago_controllers.controllers.controllers import Controllers

def get_narrow_space():
        w = Waypoint()
        warped, analytics, M = w.get_lift_information()
        s = NarrowSpaceNavSrv()
        occupancy_array = warped
        thresh = np.mean(occupancy_array.flatten())
        occupancy_array[occupancy_array < thresh] = 0  # Black regions - Free space
        occupancy_array[occupancy_array >= thresh] = 100  # White regions - Occupied

        # get the min point to go to
        p = s.choose_target_point(occupancy_array)
        # get the global point
        global_points = w.local_to_global_points(M=M, points=p)
        # get tiago there
        p = Pose()
        p.position.x = global_points[0][0]
        p.position.y = global_points[0][1]
        p.orientation.w = 1
        c = Controllers()
        c.base_controller.sync_to_pose(p)


def height_map_service(points, warped):
    rospy.wait_for_service('/input_to_location')
    try:
        req = HeightMapRequest()
        req.points = points
        req.warped = warped
        req.timestamp = rospy.Time.now()
        height_map_srv = rospy.ServiceProxy('', HeightMap)
        resp = height_map_srv(req)
        print(resp)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == '__main__':
    rospy.init_node('narrow_space_nav')
    try:
        print("narrow_space_nav")
        get_narrow_space()
    except rospy.ROSInterruptException:
        pass
