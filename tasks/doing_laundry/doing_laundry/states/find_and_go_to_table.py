import yasmin
import yasmin_ros
import rclpy

from typing import List
from geometry_msgs.msg import Point, Pose, PoseStamped, PointStamped
from std_msgs.msg import Header
from shapely import Polygon as ShapelyPolygon

from lasr_skills import Say, GoToLocation
from doing_laundry.states.compute_approach import ComputeApproach

from lasr_skills import DetectAllInPolygon


class FindAndGoToTable(yasmin.StateMachine):
    """
    Finds the dining table by detecting it within a search polygon,
    computes reachable approach poses, and navigates to the closest one.

    Ported from ROS 1 SMACH FindAndGoToTable + GoToTable.
    The two nested state machines are kept as one YASMIN StateMachine —
    GoToTable's pose-popping loop is handled inside the GO_TO_TABLE state
    via a CbState callback.

    Reads from ROS 2 params:
        doing_laundry.table.search_polygon  — list of [x, y] pairs

    Blackboard outputs:
        table_pose            : Pose   — the approach pose the robot reached
        table_approach_poses  : List[Pose]
        table_candidate_poses : List[Point]
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)
        self.add_output_key("table_pose")

        node = yasmin_ros.get_node()

        # Load search polygon from ROS 2 params
        # TODO: confirm param name matches your yaml
        try:
            raw = (
                node.get_parameter("doing_laundry.table.search_polygon")
                .get_parameter_value()
                .double_array_value
            )
            coords = list(zip(raw[::2], raw[1::2]))
            search_polygon = ShapelyPolygon(coords)
        except Exception:
            yasmin.YASMIN_LOG_WARN(
                "Could not load table search polygon from params. "
                "Using empty polygon — detection will find nothing."
            )
            search_polygon = ShapelyPolygon()

        # ── States ────────────────────────────────────────────────────────────

        self.add_state(
            "SAY_LOOKING",
            Say(text="I am looking for the table."),
            transitions={
                "succeeded": "DETECT_TABLE",
                "failed": "DETECT_TABLE",
                "aborted": "DETECT_TABLE",
            },
        )

        self.add_state(
            "DETECT_TABLE",
            DetectAllInPolygon(
                polygon=search_polygon,
                object_filter=["dining table", "tv"],
                min_confidence=0.05,
            ),
            transitions={
                "succeeded": "GET_TABLE_POSE",
                "failed": "DETECT_TABLE",  # retry on failure
            },
        )

        # Extract Point positions from detected objects
        get_table_pose_cb = yasmin.CbState(
            outcomes=["succeeded", "failed"],
            callback=self._get_table_pose,
        )
        get_table_pose_cb.add_input_key("detected_objects")
        get_table_pose_cb.add_output_key("table_candidate_poses")

        self.add_state(
            "GET_TABLE_POSE",
            get_table_pose_cb,
            transitions={
                "succeeded": "COMPUTE_APPROACH",
                "failed": "DETECT_TABLE",
            },
        )

        self.add_state(
            "COMPUTE_APPROACH",
            ComputeApproach(),
            transitions={
                "succeeded": "GO_TO_TABLE",
                "failed": "DETECT_TABLE",
            },
        )

        # Navigate to each approach pose, popping from the list until one succeeds
        go_to_table_cb = yasmin.CbState(
            outcomes=["succeeded", "failed"],
            callback=self._go_to_table,
        )
        go_to_table_cb.add_input_key("table_approach_poses")
        go_to_table_cb.add_output_key("table_approach_poses")
        go_to_table_cb.add_output_key("table_pose")

        self.add_state(
            "GO_TO_TABLE",
            go_to_table_cb,
            transitions={
                "succeeded": "succeeded",
                "failed": "DETECT_TABLE",
            },
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _get_table_pose(self, blackboard) -> str:
        """
        Extracts Point positions from detected objects, filtering for
        dining table or tv detections. Mirrors the ROS 1 _get_table_pose CBState.
        """
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
        """
        Pops approach poses one at a time and attempts navigation to each.
        Stores the successful pose as table_pose.

        Mirrors the GoToTable nested state machine from the ROS 1 version —
        the pose-popping loop is collapsed into a single callback here since
        YASMIN CbState + GoToLocation handle this more cleanly than a
        nested StateMachine.
        """
        approach_poses: List[Pose] = blackboard["table_approach_poses"]

        if not approach_poses:
            yasmin.YASMIN_LOG_WARN("No approach poses left to try.")
            return "failed"

        node = yasmin_ros.get_node()

        while approach_poses:
            pose = approach_poses.pop(0)
            blackboard["table_approach_poses"] = approach_poses

            yasmin.YASMIN_LOG_INFO(
                f"Trying approach pose at "
                f"({pose.position.x:.2f}, {pose.position.y:.2f})."
            )

            # Use GoToLocation skill — expects blackboard["location"] as PoseStamped
            # We set it temporarily and call navigate directly via Nav2
            # TODO: confirm GoToLocation's blackboard key in your ROS 2 port
            try:
                from nav2_msgs.action import NavigateToPose
                from rclpy.action import ActionClient

                client = ActionClient(node, NavigateToPose, "navigate_to_pose")
                if not client.wait_for_server(timeout_sec=5.0):
                    yasmin.YASMIN_LOG_ERROR("Nav2 action server not available.")
                    continue

                goal = NavigateToPose.Goal()
                goal.pose = PoseStamped(
                    header=Header(frame_id="map"),
                    pose=pose,
                )

                future = client.send_goal_async(goal)
                rclpy.spin_until_future_complete(node, future)
                goal_handle = future.result()

                if not goal_handle or not goal_handle.accepted:
                    yasmin.YASMIN_LOG_WARN(
                        "Navigation goal rejected, trying next pose."
                    )
                    continue

                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(node, result_future)

                blackboard["table_pose"] = pose
                yasmin.YASMIN_LOG_INFO("Successfully navigated to table.")
                return "succeeded"

            except Exception as e:
                yasmin.YASMIN_LOG_WARN(
                    f"Navigation attempt failed: {e}. Trying next pose."
                )
                continue

        yasmin.YASMIN_LOG_WARN("All approach poses exhausted.")
        return "failed"
