#!/usr/bin/env python3
import math
import rospy
import rosservice

from ..quaternion import align_poses

from nav_msgs.srv import GetPlan
from unsafe_traversal.srv import DeterminePathViability

MOVE_BASE_TOLERANCE = 0.01
PLAN_TOLERANCE = 0.1

class ViablePlanCheckerService:
    '''
    Service for calling move_base to determine whether a plan is viable between two points.

    In these sense that, the plan given by move_base is similar to the distance
    between two points so that we are not trying to route around the problem.
    '''

    # proxy: rospy.ServiceProxy
    # service: rospy.Service

    def __init__(self):
        service_list = rosservice.get_service_list()
        target_service = None

        # search for available candidates
        for service in service_list:
            if service.startswith("/move_base/") and service.endswith("/make_plan"):
                target_service = service
                break
        
        # raise an error if none found
        if target_service is None:
            raise Exception('Could not find move_base make_plan service!')

        self.proxy = rospy.ServiceProxy(target_service, GetPlan)

        # start the service
        self.service = rospy.Service('/unsafe_traversal/check_if_plan_is_viable',
                                     DeterminePathViability, self.check_plan)

    def check_plan(self, request):
        '''
        Callback for plan viability service
        '''

        # remove any need for move_base to rotate
        align_poses(request.start_pose, request.end_pose)

        # create the plan
        plan = self.proxy(request.start_pose, request.end_pose, MOVE_BASE_TOLERANCE).plan

        # calculate the distance of the plan
        dist = 0
        for cur, next in zip(plan.poses, plan.poses[1:]):
            dist += self.euclidian_distance(cur.pose.position, next.pose.position)

        # calculate our target distance
        max_dist = self.euclidian_distance(request.start_pose.pose.position, request.end_pose.pose.position)

        # check if this is within acceptable bounds
        diff = abs(dist - max_dist)
        if diff < PLAN_TOLERANCE:
            return True, diff

        return False, diff

    def euclidian_distance(self, a, b):
        return math.sqrt(
            math.pow(a.x - b.x, 2) +
            math.pow(a.y - b.y, 2) +
            math.pow(a.z - b.z, 2)
        )
