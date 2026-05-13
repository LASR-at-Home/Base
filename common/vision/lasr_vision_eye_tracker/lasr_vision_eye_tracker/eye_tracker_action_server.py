import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
import message_filters
import threading
from typing import Tuple, Optional

# ROS2 message imports (same as ROS1, just rclpy instead of rospy)
from lasr_vision_interfaces.action import (
    EyeTracker as EyeTrackerAction,
)
from lasr_vision_interfaces.srv import (
    YoloPoseDetection3D,
)
from control_msgs.srv import (
    QueryTrajectoryState,
)
from control_msgs.action import (
    PointHead,
    FollowJointTrajectory,
)
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import (
    PointStamped,
    Point,
    PoseWithCovarianceStamped,
)
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header


class EyeTracker(Node):
    def __init__(self, max_eye_distance: float = 1.5):
        super().__init__("eye_tracker_action_server")

        # Humble deadlock avoidance: callbacks that make blocking service/action calls
        # must not share one mutually-exclusive group with their done-callbacks.
        self._action_cb_group = MutuallyExclusiveCallbackGroup()
        self._work_cb_group = ReentrantCallbackGroup()

        self._done: bool = False
        self._eyes: Optional[Point] = None
        self._robot_point: Optional[Point] = None
        self._max_eye_distance: float = max_eye_distance
        self._move_up_count: float = 0.0
        self._max_move_up_count: int = 2
        self._robot_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/robot_pose",
            self._robot_pose_callback,
            qos_profile=10,
            callback_group=self._work_cb_group,
        )
        self._yolo_keypoint_client = self.create_client(
            YoloPoseDetection3D,
            "/yolo/detect3d_pose",
            callback_group=self._work_cb_group,
        )
        while not self._yolo_keypoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for YOLO keypoint service...")

        self._head_state_client = self.create_client(
            QueryTrajectoryState,
            "/head_controller/query_state",
            callback_group=self._work_cb_group,
        )
        while not self._head_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for head state service...")

        self._head_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/head_controller/follow_joint_trajectory",
            callback_group=self._work_cb_group,
        )
        while not self._head_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(
                "Waiting for follow joint trajectory action server..."
            )

        self._head_point_action_client = ActionClient(
            self,
            PointHead,
            "/head_controller/point_head_action",
            callback_group=self._work_cb_group,
        )
        while not self._head_point_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for point head action server...")

        self._action_server = ActionServer(
            self,
            EyeTrackerAction,
            "/lasr_vision_eye_tracker/track_eyes",
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
            callback_group=self._action_cb_group,
        )

        self.get_logger().info("Eye Tracker Action Server started.")

    def _goal_callback(self, goal_request) -> GoalResponse:
        """Handle incoming goal requests."""
        self.get_logger().info("Received eye tracker goal")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        """Handle cancellation requests."""
        self.get_logger().info("Eye Tracker Action Server cancellation requested")
        return CancelResponse.ACCEPT

    def _robot_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """Callback for the robot pose subscriber."""
        self._robot_point = msg.pose.pose.position

    def _get_head_join_values(self) -> Optional[Tuple[float, float]]:
        """Returns the x,y position of the head joints."""
        try:
            request = QueryTrajectoryState.Request()
            request.time = self.get_clock().now().to_msg()

            response = self._head_state_client.call(request)
            if response is None or len(response.position) < 2:
                self.get_logger().warn("Head state response was empty or invalid.")
                return None
            return (response.position[0], response.position[1])
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return None

    def _wait_for_future_result(self, future, timeout_sec: float, what: str):
        done_event = threading.Event()
        future.add_done_callback(lambda _: done_event.set())

        if not done_event.wait(timeout_sec):
            self.get_logger().error(f"Timed out waiting for {what}.")
            return None

        try:
            return future.result()
        except Exception as e:
            self.get_logger().error(f"{what} failed: {e}")
            return None

    def _look_centre(self) -> None:
        """Moves the head to look at the centre position."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]  # Look Center
        point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()
        goal.trajectory.points.append(point)
        send_goal_future = self._head_action_client.send_goal_async(goal)
        goal_handle = self._wait_for_future_result(
            send_goal_future,
            timeout_sec=2.0,
            what="centre head goal response",
        )
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Centre head goal was not accepted.")

    def _move_head_up(
        self, current_head_position: Tuple[float, float], y_delta: float = 0.25
    ) -> None:
        """Moves the head up by a certain amount.

        Args:
            current_head_position (Tuple[float, float]): The current head joint values.
            y_delta (float): The amount to move the head up by. Defaults to 0.25.
        """
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        if self._move_up_count >= self._max_move_up_count:
            self._move_up_count = 0
            point.positions = [0.0, 0.0]  # Look Center
        else:
            point.positions = [
                current_head_position[0],
                current_head_position[1] + y_delta,
            ]
        point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()
        goal.trajectory.points.append(point)

        send_goal_future = self._head_action_client.send_goal_async(goal)
        goal_handle = self._wait_for_future_result(
            send_goal_future,
            timeout_sec=2.0,
            what="move head up goal response",
        )
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Move-head-up goal was not accepted.")

        self._move_up_count += 1

    def _execute_callback(self, goal_handle):
        """Execute the eye tracking goal."""
        self.get_logger().info("Beginning eye tracking...")

        goal = goal_handle.request
        if self._robot_point is None:
            self.get_logger().warn(
                "No /robot_pose received yet; continuing and waiting asynchronously."
            )

        # First, look to person_point
        if goal.person_point is None:
            self.get_logger().error("No person point provided in goal.")
            goal_handle.abort()
            return EyeTrackerAction.Result()

        g = PointHead.Goal(
            pointing_frame="head_2_link",
            pointing_axis=Point(x=1.0, y=0.0, z=0.0),
            max_velocity=1.0,
            target=PointStamped(
                header=Header(frame_id="map"),
                point=goal.person_point,
            ),
        )

        # Send point head goal and wait
        send_goal_future = self._head_point_action_client.send_goal_async(g)
        goal_response = self._wait_for_future_result(
            send_goal_future,
            timeout_sec=2.0,
            what="initial point-head goal response",
        )
        if goal_response is None or not goal_response.accepted:
            self.get_logger().warn("Initial point-head goal was not accepted.")

        def detect_cb(image: Image, depth_image: Image, depth_camera_info: CameraInfo):
            """Callback for detection from synced messages."""
            req = YoloPoseDetection3D.Request(
                image_raw=image,
                depth_image=depth_image,
                depth_camera_info=depth_camera_info,
                model="yolo11n-pose.pt",
                confidence=0.5,
                target_frame="map",
            )

            try:
                response = self._yolo_keypoint_client.call(req)
            except Exception as e:
                self.get_logger().error(f"YOLO service call failed: {e}")
                return

            detected_keypoints = response.detections
            left_eye_point = None
            right_eye_point = None
            if not detected_keypoints:
                self._eyes = None
                return
            if not self._robot_point:
                self._eyes = None
                return
            closest_eye_midpoint = None
            closest_distance = self._max_eye_distance
            for det in detected_keypoints:
                eye_midpoint = None
                for keypoint in det.keypoints:
                    if keypoint.keypoint_name == "left_eye":
                        left_eye_point = keypoint.point
                    elif keypoint.keypoint_name == "right_eye":
                        right_eye_point = keypoint.point

                    if left_eye_point and right_eye_point:
                        # Calculate the midpoint of the two eyes
                        midpoint_x = (left_eye_point.x + right_eye_point.x) / 2.0
                        midpoint_y = (left_eye_point.y + right_eye_point.y) / 2.0
                        midpoint_z = (left_eye_point.z + right_eye_point.z) / 2.0

                        eye_midpoint = Point(x=midpoint_x, y=midpoint_y, z=midpoint_z)
                    elif left_eye_point:
                        eye_midpoint = Point(
                            x=left_eye_point.x, y=left_eye_point.y, z=left_eye_point.z
                        )
                    elif right_eye_point:
                        eye_midpoint = Point(
                            x=right_eye_point.x,
                            y=right_eye_point.y,
                            z=right_eye_point.z,
                        )
                if eye_midpoint is not None:
                    # Calculate the distance from the robot point to the eye midpoint
                    distance = (
                        (eye_midpoint.x - self._robot_point.x) ** 2
                        + (eye_midpoint.y - self._robot_point.y) ** 2
                    ) ** 0.5
                    if distance < closest_distance:
                        closest_distance = distance
                        closest_eye_midpoint = eye_midpoint
            if closest_eye_midpoint is not None:
                self._eyes = closest_eye_midpoint

        image_sub = message_filters.Subscriber(
            self,
            Image,
            "/head_front_camera/rgb/image_raw",
        )
        depth_sub = message_filters.Subscriber(
            self,
            Image,
            "/head_front_camera/depth/image_raw",
        )
        depth_camera_info_sub = message_filters.Subscriber(
            self,
            CameraInfo,
            "/head_front_camera/depth/camera_info",
        )
        ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub, depth_camera_info_sub], 10, 0.1
        )
        ts.registerCallback(detect_cb)

        self._done = False
        while rclpy.ok() and not self._done:
            if self._eyes is None:
                current_head_position = self._get_head_join_values()
                if current_head_position is None:
                    continue
                # Move the head up by a small delta
                self._move_head_up(current_head_position, y_delta=0.25)
            else:
                g = PointHead.Goal(
                    pointing_frame="head_2_link",
                    pointing_axis=Point(x=1.0, y=0.0, z=0.0),
                    max_velocity=1.0,
                    target=PointStamped(
                        header=Header(frame_id="map"),
                        point=self._eyes,
                    ),
                )
                send_goal_future = self._head_point_action_client.send_goal_async(g)
                goal_response = self._wait_for_future_result(
                    send_goal_future,
                    timeout_sec=2.0,
                    what="tracking point-head goal response",
                )
                if goal_response is None or not goal_response.accepted:
                    self.get_logger().warn("Tracking point-head goal was not accepted.")

            if goal_handle.is_cancel_requested:
                self.get_logger().info(
                    "Eye Tracker Action Server preempted, stopping tracking."
                )
                self._look_centre()
                goal_handle.canceled()
                image_sub.unregister()
                depth_sub.unregister()
                depth_camera_info_sub.unregister()
                self._done = True
                return EyeTrackerAction.Result()

            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.25))

        goal_handle.succeed()
        image_sub.unregister()
        depth_sub.unregister()
        depth_camera_info_sub.unregister()
        return EyeTrackerAction.Result()


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)

    # Create the node
    eye_tracker = EyeTracker()

    # This allows the action server to handle concurrent goals
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(eye_tracker)

    try:
        # Spin the executor
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown
        executor.shutdown()
        eye_tracker.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
