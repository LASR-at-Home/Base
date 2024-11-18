import math
from itertools import permutations
from typing import List, Union

import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker, MarkerArray


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


def is_point_free(occupancy_grid: OccupancyGrid, x: float, y: float) -> bool:
    if np.isnan(x) or np.isnan(y):
        return False
    col = int(
        (x - occupancy_grid.info.origin.position.x) / occupancy_grid.info.resolution
    )
    row = int(
        (y - occupancy_grid.info.origin.position.y) / occupancy_grid.info.resolution
    )

    if (
        col > 0
        or col >= occupancy_grid.info.width
        or row < 0
        or row >= occupancy_grid.info.height
    ):
        return False

    idx = row * occupancy_grid.info.width + col
    return occupancy_grid.data[idx] == 0


def get_approach_pose_on_radius(
    origin: PoseStamped,
    target: PoseStamped,
    radius: float,
    tolerance: float = 0.5,
    num_candidates: int = 36,
) -> Union[None, PoseStamped]:
    marker_pub = rospy.Publisher(
        "/approach_pose_candidates", MarkerArray, queue_size=10
    )
    target_pub = rospy.Publisher("/approach_pose_target", Marker, queue_size=10)
    chosen_marker_pub = rospy.Publisher("/approach_pose_chosen", Marker, queue_size=10)

    make_plan: rospy.ServiceProxy = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
    try:
        make_plan.wait_for_service(timeout=rospy.Duration.from_sec(10.0))
    except rospy.ROSException:
        rospy.loginfo("Service /move_base/make_plan not available.")
        return None

    costmap = rospy.wait_for_message("/move_base/global_costmap/costmap", OccupancyGrid)

    best_pose = None
    min_distance = float("inf")
    marker_array = MarkerArray()

    for i in range(num_candidates):
        angle = 2 * math.pi * i / num_candidates
        x = target.pose.position.x + radius * math.cos(angle)
        y = target.pose.position.y + radius * math.sin(angle)

        if not is_point_free(costmap, x, y):
            continue

        candidate_pose = PoseStamped()
        candidate_pose.header.frame_id = target.header.frame_id
        candidate_pose.pose.position.x = x
        candidate_pose.pose.position.y = y

        # Create candidate marker
        candidate_marker = Marker()
        candidate_marker.header.frame_id = target.header.frame_id
        candidate_marker.type = Marker.SPHERE
        candidate_marker.action = Marker.ADD
        candidate_marker.scale.x = 0.1  # Set size
        candidate_marker.scale.y = 0.1
        candidate_marker.scale.z = 0.1
        candidate_marker.color.r = 0.0  # Candidate color (e.g., blue)
        candidate_marker.color.g = 0.0
        candidate_marker.color.b = 1.0
        candidate_marker.color.a = 1.0
        candidate_marker.pose.position.x = x
        candidate_marker.pose.position.y = y
        candidate_marker.pose.position.z = target.pose.position.z
        candidate_marker.id = i  # Unique ID for each marker

        marker_array.markers.append(candidate_marker)

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
    marker_pub.publish(marker_array)
    if best_pose:
        chosen_marker = Marker()
        chosen_marker.header.frame_id = best_pose.header.frame_id
        chosen_marker.type = Marker.SPHERE
        chosen_marker.action = Marker.ADD
        chosen_marker.scale.x = 0.15  # Set larger size for the chosen pose
        chosen_marker.scale.y = 0.15
        chosen_marker.scale.z = 0.15
        chosen_marker.color.r = 1.0  # Chosen color (e.g., red)
        chosen_marker.color.g = 0.0
        chosen_marker.color.b = 0.0
        chosen_marker.color.a = 1.0
        chosen_marker.pose = best_pose.pose
        chosen_marker.id = 999  # Unique ID for chosen marker
        chosen_marker_pub.publish(chosen_marker)

    target_marker = Marker()
    target_marker.header.frame_id = target.header.frame_id
    target_marker.type = Marker.SPHERE
    target_marker.action = Marker.ADD
    target_marker.scale.x = 0.15  # Set larger size for the chosen pose
    target_marker.scale.y = 0.15
    target_marker.scale.z = 0.15
    target_marker.color.r = 1.0  # Chosen color (e.g., red)
    target_marker.color.g = 0.0
    target_marker.color.b = 0.0
    target_marker.color.a = 1.0
    target_marker.pose = target.pose
    target_marker.id = 999  # Unique ID for chosen marker
    target_pub.publish(target_marker)

    return best_pose


def compute_face_quat(p1: Pose, p2: Pose) -> Quaternion:
    dx: float = p2.position.x - p1.position.x
    dy: float = p2.position.y - p1.position.y
    theta_deg = np.degrees(math.atan2(dy, dx))
    x, y, z, w = R.from_euler("z", theta_deg, degrees=True).as_quat()
    return Quaternion(x, y, z, w)
