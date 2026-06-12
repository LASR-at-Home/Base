import time
import numpy as np
import yasmin
import yasmin_ros
from yasmin_ros.yasmin_node import YasminNode

from typing import List, Optional
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header

from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient


def _wait_future(future, timeout):
    deadline = time.time() + timeout
    while not future.done() and time.time() < deadline:
        time.sleep(0.02)
    return future.result() if future.done() else None


class ComputeApproach(yasmin.State):
    """Computes reachable approach poses around each detected table candidate
    using Nav2 ComputePathToPose. Ported from ROS1 ComputeApproach."""

    def __init__(self, map_frame_min_distance: float = 1.0, n_samples: int = 25):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("table_candidate_poses")
        self.add_output_key("table_approach_poses")

        self.node = yasmin_ros.logger_node
        self._map_frame_min_distance = map_frame_min_distance
        self.n_samples = n_samples

        self._path_client = ActionClient(
            self.node, ComputePathToPose, "/compute_path_to_pose"
        )

        self._amcl_pose: Optional[Pose] = None
        self.node.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10
        )

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._amcl_pose = msg.pose.pose

    def execute(self, blackboard) -> str:
        table_candidates: List[Point] = blackboard["table_candidate_poses"]
        if not table_candidates:
            yasmin.YASMIN_LOG_WARN("No table candidate poses in blackboard.")
            return "failed"

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
                    f"No reachable approach pose for candidate "
                    f"({table_point.x:.2f}, {table_point.y:.2f})."
                )

        if approach_poses:
            blackboard["table_approach_poses"] = approach_poses
            yasmin.YASMIN_LOG_INFO(f"Computed {len(approach_poses)} approach poses.")
            return "succeeded"

        yasmin.YASMIN_LOG_WARN("No reachable approach poses found.")
        return "failed"

    def _get_current_pose(self) -> Optional[Pose]:
        deadline = time.time() + 5.0
        while self._amcl_pose is None and time.time() < deadline:
            time.sleep(0.05)
        if self._amcl_pose is None:
            yasmin.YASMIN_LOG_WARN("No /amcl_pose received, using origin.")
            return Pose()
        return self._amcl_pose

    def _sample_points(self, point: Point) -> List[Point]:
        angles = np.linspace(0, 2 * np.pi, self.n_samples, endpoint=False)
        return [
            Point(
                x=point.x + self._map_frame_min_distance * np.cos(a),
                y=point.y + self._map_frame_min_distance * np.sin(a),
                z=0.0,
            )
            for a in angles
        ]

    def _calculate_poses(self, target: Point, samples: List[Point]) -> List[Pose]:
        poses = []
        for p in samples:
            angle = np.arctan2(target.y - p.y, target.x - p.x)
            pose = Pose()
            pose.position = p
            pose.orientation.z = np.sin(angle / 2)
            pose.orientation.w = np.cos(angle / 2)
            poses.append(pose)
        return poses

    def _can_reach_pose(self, target_pose: Pose) -> bool:
        if not self._path_client.wait_for_server(timeout_sec=3.0):
            yasmin.YASMIN_LOG_WARN("ComputePathToPose server not available.")
            return False

        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped(header=Header(frame_id="map"), pose=target_pose)
        goal.planner_id = ""

        try:
            goal_handle = _wait_future(self._path_client.send_goal_async(goal), 3.0)
            if goal_handle is None or not goal_handle.accepted:
                return False
            result = _wait_future(goal_handle.get_result_async(), 5.0)
            return result is not None and len(result.result.path.poses) > 0
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"Path planning check failed: {e}")
            return False