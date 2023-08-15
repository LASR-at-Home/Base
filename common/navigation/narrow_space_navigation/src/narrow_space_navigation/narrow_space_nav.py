#!/usr/bin/env python3
from waypoints import *
from narrow_space_navigation.srv import HeightMap, HeightMapResponse, HeightMapRequest
from narrow_space_nav_srv import NarrowSpaceNavSrv
from tiago_controllers.controllers.controllers import Controllers

# # get from rosparam
# lift_points = {
#     real_lift : get_real_lift,
#     get_sim_lift : get_sim_lift,
#     get_real_test_lift : get_real_test_lift
# }
# if not is_sim:
        #     elevator_center = 3.66016769409, 0.0551229566336
        #     points = [[2.12485527992, 1.20056855679],
        #               [2.4371213913, -0.712846040726],
        #               [4.71406841278, -0.426484644413],
        #               [4.14157915115, 1.20056855679]]
        # else:
        #     elevator_center = 3.66016769409, 0.0551229566336
        #     points = [[3.23080015182, -1.33763432503],
        #               [4.57094621658, -1.10333895683],
        #               [5.05235815048, -2.76944160461],
        #               [3.45198941231, -3.251049757]]

def get_narrow_space(eis_with_local_cstmap=False):
        """
        Get the window of the elevator.

        :param window: a window of the costmap

        :return: a window of the elevator
        """
        w = Waypoint()
        warped, analytics, M = w.get_lift_information()
        s = NarrowSpaceNavSrv()
        # p = s.process_occupancy_grid(warped, analytics)
        occupancy_array = warped
        # Convert grayscale image to binary occupancy grid
        thresh = np.mean(occupancy_array.flatten())
        occupancy_array[occupancy_array < thresh] = 0  # Black regions - Free space
        occupancy_array[occupancy_array >= thresh] = 100  # White regions - Occupied
        p = s.choose_target_point(occupancy_array)

        print(p)
        print(f"the analythics are {analytics}")

        global_points = w.local_to_global_points(M=M, points=p)
        p = Pose()
        p.position.x = global_points[0][0]
        p.position.y = global_points[0][1]
        p.orientation.w = 1
        c = Controllers()
        c.base_controller.sync_to_pose(p)


        # points = []
        # req = HeightMapRequest()
        # req.warped = list(warped)
        # req.points = points
        # req.timestamp = rospy.Time.now()
        # height_map_srv = rospy.ServiceProxy('/narrow_space_nav_srv', HeightMap)
        # resp = height_map_srv(req)
        # return

        # points = self.find_free_location_in_elevator(M=M, warped=warped, elevator_number=elevator_number, is_cnt=False)

        # get to the point in the lift
        # p = Pose()
        # p.position.x = points[0][0]
        # p.position.y = points[0][1]
        # p.orientation.w = 1
        # self.base_controller.sync_to_pose(p)


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
