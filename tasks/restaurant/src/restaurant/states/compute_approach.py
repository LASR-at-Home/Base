import smach
import rospy


import numpy as np

from typing import List
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path

"""This might all be overkill -- movebase's tolerance could be set instead to just do 
everything with the planner"""


class ComputeApproach(smach.State):
    """Computes the robot's approach pose given the point of the
    detected waving cusotmer to go to"""

    # Minimum distance in the map frame the robot should be
    _map_frame_min_distance: float
    # Number of samples to use to get the approach pose
    n_samples: int
    # Movebase planner
    _movebase_planner: rospy.ServiceProxy

    def __init__(self, map_frame_min_distance: float = 0.5, n_samples: int = 25):
        """
        Args:
            map_frame_min_distance (float, optional): minimum (Eucl)
            distance in the map frame between the robot and the customer.
            Defaults to 0.5.

            n_samples (int, optional): number of samples to use to get the
            approach pose. Defaults to 25.
        """
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["waving_person_detection"],
            output_keys=["customer_approach_pose"],
        )

        self._map_frame_min_distance = map_frame_min_distance
        self.n_samples = n_samples
        self._movebase_planner = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
        self._movebase_planner.wait_for_service()

    def _sample_points(self, point: Point) -> List[Point]:
        """Samples points uniformly around the circumference of a circle
        with radius `self._map_frame_min_distance` around the given point.

        Args:
            point (Point): Centre of the circle

        Returns:
            List[Point]: List of points sampled uniformly around the
            circumference of the circle.


        Reference:
        https://stackoverflow.com/questions/48358792/generating-random-points-on-the-circumference-of-a-circle-python
        """

        angles = np.linspace(0, 2 * np.pi, self.n_samples, endpoint=False)
        return [
            Point(
                x=point.x + self._map_frame_min_distance * np.cos(angle),
                y=point.y + self._map_frame_min_distance * np.sin(angle),
                z=0.0,
            )
            for angle in angles
        ]

    def _calculate_poses(
        self, person_point: Point, point_samples: List[Point]
    ) -> List[Pose]:
        """Calulates the approach poses for the robot to go to, such that the robot is
        facing the person at each of the sampled points.

        Args:
            person_point (Point): Point of the detected waving person

            point_samples (List[Point]): List of points sampled uniformly around the
            circumference of a circle with radius `self._map_frame_min_distance`
            around the given point.

        Returns:
            List[Pose]: List of poses for the robot to go to, such that the robot is
            facing the person at each of the sampled points.
        """

        poses = []
        for point in point_samples:
            # Calculate the angle to the person
            dx = person_point.x - point.x
            dy = person_point.y - point.y
            angle = np.arctan2(dy, dx)

            # Create a pose with the robot facing the person
            pose = Pose()
            pose.position = point
            pose.orientation.z = np.sin(angle / 2)
            pose.orientation.w = np.cos(angle / 2)
            poses.append(pose)

        return poses

    def _can_reach_pose(self, current_pose: Pose, target_pose: Pose) -> bool:
        """Checks if the robot can reach the given pose, using movebase

        Args:
            pose (Pose): Pose to check

        Returns:
            bool: True if the robot can reach the pose, False otherwise.
        """

        try:
            current_pose_stamped = PoseStamped()
            current_pose_stamped.header.frame_id = "map"
            current_pose_stamped.pose = current_pose

            target_pose_stamped = PoseStamped()
            target_pose_stamped.header.frame_id = "map"
            target_pose_stamped.pose = target_pose

            plan = self._movebase_planner(
                start=current_pose_stamped,
                goal=target_pose_stamped,
                tolerance=0.1,
            ).plan
            return len(plan.poses) > 0
        except rospy.ServiceException as e:
            rospy.logerr("Movebase Planner Service call failed: %s", str(e))
            return False

    def execute(self, userdata):
        current_pose = rospy.wait_for_message(
            "/robot_pose", PoseWithCovarianceStamped
        ).pose.pose
        customer_point = userdata.waving_person_detection.point

        point_samples = self._sample_points(customer_point)
        approach_poses = self._calculate_poses(customer_point, point_samples)

        for pose in approach_poses:
            if self._can_reach_pose(current_pose, pose):
                userdata.customer_approach_pose = pose
                rospy.loginfo(
                    "Computed approach pose: %s", userdata.customer_approach_pose
                )
                return "succeeded"
        rospy.logwarn("No reachable approach pose found.")
        return "failed"
