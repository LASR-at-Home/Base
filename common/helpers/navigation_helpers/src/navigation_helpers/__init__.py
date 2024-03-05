from geometry_msgs.msg import (
    Point,
    PoseWithCovarianceStamped,
    Pose,
    Quaternion,
    PoseStamped,
)
from nav_msgs.srv import GetPlan
import rospy

from typing import Union, List, Tuple


def points_on_radius(point: Point, radius: float = 0.5) -> List[Point]:
    return [
        Point(x=point.x + 0.0, y=point.y - radius, z=point.z),
        Point(x=point.x + 0.0, y=point.y + radius, z=point.z),
        Point(x=point.x - radius, y=point.y + 0.0, z=point.z),
        Point(x=point.x + radius, y=point.y + 0.0, z=point.z),
        Point(x=point.x + radius, y=point.y + radius, z=point.z),
        Point(x=point.x - radius, y=point.y - radius, z=point.z),
        Point(x=point.x - radius, y=point.y + radius, z=point.z),
        Point(x=point.x + radius, y=point.y - radius, z=point.z),
    ]


def euclidian_distance(p1: Point, p2: Point) -> float:
    return ((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2) ** 0.5


def make_plan(
    robot_pose: PoseWithCovarianceStamped,
    point: Point,
    tol: float = 0.01,
    max_dist: Union[float, None] = None,
) -> Tuple[bool, Union[Pose, None], Union[List[PoseStamped], None], float]:
    start = PoseStamped(pose=robot_pose.pose.pose, header=robot_pose.header)
    goal = PoseStamped(
        pose=Pose(position=point, orientation=robot_pose.pose.pose.orientation),
        header=robot_pose.header,
    )
    make_plan = rospy.ServiceProxy("/move_base/NavfnROS/make_plan", GetPlan)
    plan = make_plan(start, goal, tol).plan
    if len(plan.poses) > 0:
        if max_dist is not None:
            dist = 0.0
            for current, next in zip(plan.poses, plan.poses[1:]):
                dist += euclidian_distance(current.pose.position, next.pose.position)
            if dist > max_dist:
                return False, None, None, dist
        return True, plan.poses[-1].pose, plan.poses, dist
    return False, None, None


def plan_to_radius(
    robot_pose: PoseWithCovarianceStamped,
    point: Point,
    radius: float = 0.5,
    tol: float = 0.1,
) -> List[PoseStamped]:
    picked_points = []
    for p in points_on_radius(point, radius=radius):
        success, final_p, _, dist = make_plan(robot_pose, p, tol)
        if success:
            picked_points.append([final_p, dist])

    picked_points = sorted(picked_points, key=lambda x: x[1])
    return [point[0] for point in picked_points] if picked_points else []
