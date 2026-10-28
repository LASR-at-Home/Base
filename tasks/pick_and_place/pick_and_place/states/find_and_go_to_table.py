import time
import yasmin
import yasmin_ros
from yasmin_ros.yasmin_node import YasminNode

from typing import List
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header
from shapely import Polygon as ShapelyPolygon

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from lasr_skills import Say, DetectAllInPolygon
from pick_and_place.states.compute_approach import ComputeApproach


def _wait_future(future, timeout):
    deadline = time.time() + timeout
    while not future.done() and time.time() < deadline:
        time.sleep(0.02)
    return future.result() if future.done() else None


class FindAndGoToTable(yasmin.StateMachine):
    """Finds the table in a search polygon, computes reachable approach poses,
    and navigates to the closest one. Ported from ROS1 FindAndGoToTable+GoToTable."""

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)
        self.add_output_key("table_pose")

        node = yasmin_ros.logger_node

        try:
            raw = (
                node.get_parameter("pick_and_place.table.search_polygon")
                .get_parameter_value()
                .double_array_value
            )
            coords = list(zip(raw[::2], raw[1::2]))
            search_polygon = ShapelyPolygon(coords)
        except Exception:
            yasmin.YASMIN_LOG_WARN(
                "Could not load table search polygon — using empty polygon."
            )
            search_polygon = ShapelyPolygon()

        self.add_state(
            "SAY_LOOKING",
            Say(text="I am looking for the table."),
            transitions={"succeeded": "DETECT_TABLE",
                         "aborted": "DETECT_TABLE", "canceled": "DETECT_TABLE"},
        )

        self.add_state(
            "DETECT_TABLE",
            DetectAllInPolygon(
                polygon=search_polygon,
                object_filter=["dining table", "tv"],
                min_confidence=0.05,
            ),
            transitions={"succeeded": "GET_TABLE_POSE", "failed": "DETECT_TABLE"},
        )

        get_table_pose_cb = yasmin.CbState(
            outcomes=["succeeded", "failed"], callback=self._get_table_pose,
        )
        get_table_pose_cb.add_input_key("detected_objects")
        get_table_pose_cb.add_output_key("table_candidate_poses")
        self.add_state(
            "GET_TABLE_POSE", get_table_pose_cb,
            transitions={"succeeded": "COMPUTE_APPROACH", "failed": "DETECT_TABLE"},
        )

        self.add_state(
            "COMPUTE_APPROACH", ComputeApproach(),
            transitions={"succeeded": "GO_TO_TABLE", "failed": "DETECT_TABLE"},
        )

        go_to_table_cb = yasmin.CbState(
            outcomes=["succeeded", "failed"], callback=self._go_to_table,
        )
        go_to_table_cb.add_input_key("table_approach_poses")
        go_to_table_cb.add_output_key("table_approach_poses")
        go_to_table_cb.add_output_key("table_pose")
        self.add_state(
            "GO_TO_TABLE", go_to_table_cb,
            transitions={"succeeded": "succeeded", "failed": "DETECT_TABLE"},
        )

    def _get_table_pose(self, blackboard) -> str:
        table_points = [
            obj.point
            for obj in blackboard["detected_objects"]
            if obj.name in ["dining table", "tv"]
        ]
        if table_points:
            blackboard["table_candidate_poses"] = table_points
            yasmin.YASMIN_LOG_INFO(f"Found {len(table_points)} table candidate(s).")
            return "succeeded"
        yasmin.YASMIN_LOG_WARN("No dining table or tv detected in polygon.")
        return "failed"

    def _go_to_table(self, blackboard) -> str:
        approach_poses: List[Pose] = blackboard["table_approach_poses"]
        if not approach_poses:
            yasmin.YASMIN_LOG_WARN("No approach poses left to try.")
            return "failed"

        node = yasmin_ros.logger_node
        client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        if not client.wait_for_server(timeout_sec=5.0):
            yasmin.YASMIN_LOG_ERROR("Nav2 action server not available.")
            return "failed"

        while approach_poses:
            pose = approach_poses.pop(0)
            blackboard["table_approach_poses"] = approach_poses
            yasmin.YASMIN_LOG_INFO(
                f"Trying approach pose at ({pose.position.x:.2f}, {pose.position.y:.2f})."
            )
            try:
                goal = NavigateToPose.Goal()
                goal.pose = PoseStamped(header=Header(frame_id="map"), pose=pose)

                goal_handle = _wait_future(client.send_goal_async(goal), 5.0)
                if goal_handle is None or not goal_handle.accepted:
                    yasmin.YASMIN_LOG_WARN("Goal rejected, trying next pose.")
                    continue

                result = _wait_future(goal_handle.get_result_async(), 120.0)
                if result is None:
                    yasmin.YASMIN_LOG_WARN("Navigation timed out, trying next pose.")
                    continue
                if result.status != 4:
                    yasmin.YASMIN_LOG_WARN(f"Nav failed (status {result.status}), next pose.")
                    continue

                blackboard["table_pose"] = pose
                yasmin.YASMIN_LOG_INFO("Successfully navigated to table.")
                return "succeeded"
            except Exception as e:
                yasmin.YASMIN_LOG_WARN(f"Nav attempt failed: {e}. Trying next pose.")
                continue

        yasmin.YASMIN_LOG_WARN("All approach poses exhausted.")
        return "failed"