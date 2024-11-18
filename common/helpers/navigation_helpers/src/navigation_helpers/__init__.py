import math
from itertools import permutations
from typing import List, Union

import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from scipy.spatial.transform import Rotation as R


def euclidian_distance(p1: Point, p2: Point) -> float:
    return ((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2) ** 0.5


def min_hamiltonian_path(start: Pose, poses: List[Pose]) -> Union[None, List[Pose]]:
    best_order = None
    min_distance = np.inf

    for perm in permutations(poses):
        dist = euclidian_distance(start.position, perm[0].position)
        for i in range(len(poses) - 1):
            dist += euclidian_distance(perm[i].position, perm[i + 1].position)

        if dist < min_distance:
            min_distance = dist
            best_order = list(perm)

    return best_order


def get_pose_on_path(
    p1: PoseStamped, p2: PoseStamped, dist_to_goal: float = 1.0, tolerance: float = 0.5
) -> Union[None, PoseStamped]:
    make_plan: rospy.ServiceProxy = rospy.ServiceProxy("/move_base/make_plan", GetPlan)

    chosen_pose: Union[None, PoseStamped] = None

    rospy.loginfo(f"Getting plan from {p1} to {p2}.")

    if p1.header.frame_id != p2.header.frame_id != "map":
        rospy.loginfo(
            f"Frames of reference are not 'map' ({p1.header.frame_id} and {p2.header.frame_id})."
        )
        return chosen_pose

    try:
        make_plan.wait_for_service(timeout=rospy.Duration.from_sec(10.0))
    except rospy.ROSException:
        rospy.loginfo("Service /move_base/make_plan not available.")
        return chosen_pose

    try:
        plan: Path = make_plan(p1, p2, tolerance).plan
    except rospy.ServiceException as e:
        rospy.loginfo(e)
        return chosen_pose

    rospy.loginfo(f"Got plan with {len(plan.poses)} poses.")

    if len(plan.poses) > 0:
        for pose in reversed(plan.poses):
            if euclidian_distance(pose.pose.position, p2.pose.position) >= dist_to_goal:
                chosen_pose = pose
                break

    return chosen_pose


def get_approach_pose_on_radius(
    origin: PoseStamped,
    target: PoseStamped,
    radius: float,
    tolerance: float = 0.5,
    num_candidates: int = 36,
) -> Union[None, PoseStamped]:

    make_plan: rospy.ServiceProxy = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
    try:
        make_plan.wait_for_service(timeout=rospy.Duration.from_sec(10.0))
    except rospy.ROSException:
        rospy.loginfo("Service /move_base/make_plan not available.")
        return None

    best_pose = None
    min_distance = float("inf")

    for i in range(num_candidates):
        angle = 2 * math.pi * i / num_candidates
        x = target.pose.position.x + radius * math.cos(angle)
        y = target.pose.position.y + radius * math.sin(angle)

        candidate_pose = PoseStamped()
        candidate_pose.header.frame_id = target.header.frame_id
        candidate_pose.pose.position.x = x
        candidate_pose.pose.position.y = y

        plan = make_plan(origin, candidate_pose, tolerance)
        if plan.plan.poses:
            distance_from_origin = math.hypot(
                x - origin.pose.position.x, y - origin.pose.position.y
            )
            distance_from_goal = math.hypot(
                x - target.pose.position.x, y - target.pose.position.y
            )

            if (
                distance_from_origin < min_distance
                and abs(distance_from_goal - radius) <= tolerance
            ):
                min_distance = distance_from_origin
                best_pose = candidate_pose

    return best_pose


def compute_face_quat(p1: Pose, p2: Pose) -> Quaternion:
    dx: float = p2.position.x - p1.position.x
    dy: float = p2.position.y - p1.position.y
    theta_deg = np.degrees(math.atan2(dy, dx))
    x, y, z, w = R.from_euler("z", theta_deg, degrees=True).as_quat()
    return Quaternion(x, y, z, w)
