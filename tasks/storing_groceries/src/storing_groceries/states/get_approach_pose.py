import smach
import rospy
import numpy as np

from typing import List
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.srv import GetPlan


class ComputeApproach(smach.State):
    """Computes approach poses for the robot given a list of table candidate positions."""

    _map_frame_min_distance: float
    n_samples: int
    _movebase_planner: rospy.ServiceProxy

    def __init__(self, map_frame_min_distance: float = 1, n_samples: int = 25):
        """
        Args:
            map_frame_min_distance (float, optional): Minimum Euclidean distance
            to maintain from the table. Defaults to 1.0.

            n_samples (int, optional): Number of samples to generate around each table.
            Defaults to 25.
        """
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["table_candidate_poses"],
            output_keys=["table_approach_poses"],
        )

        self._map_frame_min_distance = map_frame_min_distance
        self.n_samples = n_samples
        self._movebase_planner = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
        self._movebase_planner.wait_for_service()

    def _sample_points(self, point: Point) -> List[Point]:
        """Samples points uniformly around a circle centered at the given point.

        Args:
            point (Point): Center of the circle (e.g., table location)

        Returns:
            List[Point]: Sampled points on the circle perimeter.
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
        self, target_point: Point, point_samples: List[Point]
    ) -> List[Pose]:
        """Calculates poses facing the target point (e.g., a table) from each sample.

        Args:
            target_point (Point): The point the robot should face (e.g., table center).
            point_samples (List[Point]): Points around the circle for approach.

        Returns:
            List[Pose]: Approach poses oriented toward the target point.
        """
        poses = []
        for point in point_samples:
            dx = target_point.x - point.x
            dy = target_point.y - point.y
            angle = np.arctan2(dy, dx)

            pose = Pose()
            pose.position = point
            pose.orientation.z = np.sin(angle / 2)
            pose.orientation.w = np.cos(angle / 2)
            poses.append(pose)

        return poses

    def _can_reach_pose(self, current_pose: Pose, target_pose: Pose) -> bool:
        """Checks if the robot can reach a pose using the move_base planner.

        Args:
            current_pose (Pose): Current robot pose
            target_pose (Pose): Pose to check reachability for

        Returns:
            bool: True if reachable, False otherwise
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
                tolerance=0.25,
            ).plan
            return len(plan.poses) > 0
        except rospy.ServiceException as e:
            rospy.logerr("MoveBase planner service call failed: %s", str(e))
            return False

    def execute(self, userdata):
        try:
            current_pose = rospy.wait_for_message(
                "/robot_pose", PoseWithCovarianceStamped
            ).pose.pose

            table_candidates: List[Point] = userdata.table_candidate_poses
            approach_poses: List[Pose] = []

            for table_point in table_candidates:
                point_samples = self._sample_points(table_point)
                candidate_poses = self._calculate_poses(table_point, point_samples)

                closest_distance = float("inf")
                closest_pose = None

                for pose in candidate_poses:
                    if self._can_reach_pose(current_pose, pose):
                        distance = np.linalg.norm(
                            np.array([pose.position.x, pose.position.y])
                            - np.array(
                                [current_pose.position.x, current_pose.position.y]
                            )
                        )
                        if distance < closest_distance:
                            closest_distance = distance
                            closest_pose = pose

                if closest_pose is not None:
                    approach_poses.append(closest_pose)
                else:
                    rospy.logwarn(
                        "No reachable approach pose found for one table candidate."
                    )

            if approach_poses:
                userdata.table_approach_poses = approach_poses
                rospy.loginfo(
                    "Approach poses computed for %d table candidates.",
                    len(approach_poses),
                )
                return "succeeded"
            else:
                rospy.logwarn(
                    "No reachable approach poses found for any table candidates."
                )
                return "failed"

        except Exception as e:
            rospy.logerr("Exception in ComputeApproach execute: %s", str(e))
            return "failed"
