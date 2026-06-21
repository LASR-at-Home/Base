import numpy as np
import yasmin
import yasmin_ros
import rclpy

from typing import List
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header

from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient


class ComputeApproach(yasmin.State):
    """
    Computes reachable approach poses around each detected table candidate.

    Ported from ROS 1 SMACH ComputeApproach. The move_base make_plan service
    is replaced with the Nav2 ComputePathToPose action to check reachability.

    Blackboard inputs:
        table_candidate_poses : List[Point]
            3D positions of detected table candidates.

    Blackboard outputs:
        table_approach_poses : List[Pose]
            Reachable approach poses sorted closest-first, one per table candidate.
    """

    def __init__(self, map_frame_min_distance: float = 1.0, n_samples: int = 25):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("table_candidate_poses")
        self.add_output_key("table_approach_poses")

        self.node = yasmin_ros.get_node()
        self._map_frame_min_distance = map_frame_min_distance
        self.n_samples = n_samples

        # Nav2 action client for checking path reachability
        self._path_client = ActionClient(
            self.node, ComputePathToPose, "/compute_path_to_pose"
        )

    def execute(self, blackboard) -> str:
        table_candidates: List[Point] = blackboard["table_candidate_poses"]

        if not table_candidates:
            yasmin.YASMIN_LOG_WARN("No table candidate poses in blackboard.")
            return "failed"

        # Get current robot pose
        current_pose = self._get_current_pose()
        if current_pose is None:
            yasmin.YASMIN_LOG_ERROR("Could not get current robot pose.")
            return "failed"

        approach_poses: List[Pose] = []

        for table_point in table_candidates:
            point_samples = self._sample_points(table_point)
            candidate_poses = self._calculate_poses(table_point, point_samples)

            closest_distance = float("inf")
            closest_pose = None

            for pose in candidate_poses:
                if self._can_reach_pose(pose):
                    distance = np.linalg.norm(
                        np.array([pose.position.x, pose.position.y])
                        - np.array([current_pose.position.x, current_pose.position.y])
                    )
                    if distance < closest_distance:
                        closest_distance = distance
                        closest_pose = pose

            if closest_pose is not None:
                approach_poses.append(closest_pose)
            else:
                yasmin.YASMIN_LOG_WARN(
                    f"No reachable approach pose found for table candidate "
                    f"at ({table_point.x:.2f}, {table_point.y:.2f})."
                )

        if approach_poses:
            blackboard["table_approach_poses"] = approach_poses
            yasmin.YASMIN_LOG_INFO(f"Computed {len(approach_poses)} approach poses.")
            return "succeeded"

        yasmin.YASMIN_LOG_WARN(
            "No reachable approach poses found for any table candidate."
        )
        return "failed"

    # ── Private helpers ───────────────────────────────────────────────────────

    def _get_current_pose(self) -> Pose | None:
        """
        Gets the current robot pose from /amcl_pose.
        Falls back to origin if the topic is unavailable.
        """
        try:
            success, msg = rclpy.wait_for_message.wait_for_message(
                msg_type=PoseWithCovarianceStamped,
                node=self.node,
                topic="/amcl_pose",
                time_to_wait=5.0,
            )
            if success:
                return msg.pose.pose
            yasmin.YASMIN_LOG_WARN("No pose received from /amcl_pose, using origin.")
            return Pose()
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Failed to get current pose: {e}")
            return None

    def _sample_points(self, point: Point) -> List[Point]:
        """
        Samples points uniformly around a circle of radius
        map_frame_min_distance centred at the given table point.
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
        """
        Calculates poses facing the target point from each sampled position.
        Orientation is a yaw-only quaternion pointing toward target_point.
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

    def _can_reach_pose(self, target_pose: Pose) -> bool:
        """
        Checks reachability by sending a ComputePathToPose goal to Nav2.
        Returns True if Nav2 returns a non-empty path.

        This replaces the ROS 1 move_base/make_plan service call.
        """
        if not self._path_client.wait_for_server(timeout_sec=3.0):
            yasmin.YASMIN_LOG_WARN("ComputePathToPose action server not available.")
            return False

        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped(
            header=Header(frame_id="map"),
            pose=target_pose,
        )
        goal.planner_id = ""

        try:
            future = self._path_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
            goal_handle = future.result()

            if not goal_handle or not goal_handle.accepted:
                return False

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=5.0)
            result = result_future.result()

            return result is not None and len(result.result.path.poses) > 0
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"Path planning check failed: {e}")
            return False
