import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
import message_filters
import time
import asyncio
from typing import Tuple, Optional

# ROS2 message imports (same as ROS1, just rclpy instead of rospy)
from lasr_vision_interfaces.msg import (
    EyeTrackerGoal,
    EyeTrackerResult,
    EyeTrackerAction,
)
from lasr_vision_interfaces.srv import (
    YoloPoseDetection3D,
    YoloPoseDetection3DRequest,
)
from control_msgs.srv import (
    QueryTrajectoryState,
)
from control_msgs.msg import (
    PointHeadGoal,
    PointHeadAction,
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
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
            qos_profile=10,  # ROS2 uses QoS profiles; 10 is queue size equivalent
        )
        self._yolo_keypoint_client = self.create_client(
            YoloPoseDetection3D,
            "/yolo/detect3d_pose",
        )
        self._head_action_client = ActionClient(
            self,
            FollowJointTrajectoryAction,
            "/head_controller/follow_joint_trajectory",
        )
        self._head_point_action_client = ActionClient(
            self,
            PointHeadAction,
            "/head_controller/point_head_action",
        )
        self._action_server = ActionServer(
            self,
            EyeTrackerAction,
            "/lasr_vision_eye_tracker/track_eyes",
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
        )
        self.get_logger().info("Eye Tracker Action Server started.")

    def _goal_callback(self, goal_request: EyeTrackerGoal) -> GoalResponse:
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
            # Wait for service to be available
            if not self._head_state_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("Head state service not available")
                return None

            # In ROS2, we must create the request object explicitly
            request = QueryTrajectoryState()
            request.time = (
                self.get_clock().now()
            )  # ROS2 CHANGE #10: Use self.get_clock().now()

            response = self._head_state_client.call(request)
            return response.position
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return None

    async def _look_centre(self) -> None:
        """Moves the head to look at the centre position."""
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]  # Look Center
        point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()
        goal.trajectory.points.append(point)
        send_goal_future = self._head_action_client.send_goal_async(goal)

        # Wait for goal to be accepted and get result
        goal_handle = await send_goal_future
        if not goal_handle.accepted:
            self.get_logger().error("Head look center goal rejected")
            return

        # Wait for result
        result_future = goal_handle.get_result_async()
        await result_future

    async def _move_head_up(
        self, current_head_position: Tuple[float, float], y_delta: float = 0.25
    ) -> None:
        """Moves the head up by a certain amount.

        Args:
            current_head_position (Tuple[float, float]): The current head joint values.
            y_delta (float): The amount to move the head up by. Defaults to 0.25.
        """
        goal = FollowJointTrajectoryGoal()
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
        goal_handle = await send_goal_future
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            await result_future

        self._move_up_count += 1

    async def _execute_callback(self, goal_handle) -> EyeTrackerResult:
        """Execute the eye tracking goal."""
        self.get_logger().info("Beginning eye tracking...")

        goal: EyeTrackerGoal = goal_handle.request
        try:
            # Create a one-time message future
            future = rclpy.task.Future()

            def pose_callback(msg):
                if not future.done():
                    future.set_result(msg)

            pose_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                "/robot_pose",
                pose_callback,
                qos_profile=10,
            )

            # Wait with timeout
            while rclpy.ok() and not future.done():
                await asyncio.sleep(0.01)

            self.destroy_subscription(pose_sub)

            if future.done():
                msg = future.result()
                self._robot_point = msg.pose.pose.position
        except Exception as e:
            self.get_logger().error(f"Failed to get robot pose: {e}")
            goal_handle.abort()
            return EyeTrackerResult()

        # First, look to person_point
        if goal.person_point is None:
            self.get_logger().error("No person point provided in goal.")
            goal_handle.abort()
            return EyeTrackerResult()

        g = PointHeadGoal(
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
        point_goal_handle = await send_goal_future
        if point_goal_handle.accepted:
            result_future = point_goal_handle.get_result_async()
            await result_future

        def detect_cb(image: Image, depth_image: Image, depth_camera_info: CameraInfo):
            """Callback for detection from synced messages."""
            req = YoloPoseDetection3DRequest(
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

        image_sub = message_filters.Subscriber(self, Image, "/xtion/rgb/image_raw")
        depth_sub = message_filters.Subscriber(
            self, Image, "/xtion/depth_registered/image_raw"
        )
        depth_camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, "/xtion/depth_registered/camera_info"
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
                    await asyncio.sleep(0.1)
                    continue
                # Move the head up by a small delta
                await self._move_head_up(current_head_position, y_delta=0.25)
                await asyncio.sleep(0.5)
            else:
                g = PointHeadGoal(
                    pointing_frame="head_2_link",
                    pointing_axis=Point(x=1.0, y=0.0, z=0.0),
                    max_velocity=1.0,
                    target=PointStamped(
                        header=Header(frame_id="map"),
                        point=self._eyes,
                    ),
                )
                send_goal_future = self._head_point_action_client.send_goal_async(g)
                point_goal_handle = await send_goal_future
                if point_goal_handle.accepted:
                    result_future = point_goal_handle.get_result_async()
                    await result_future

            if goal_handle.is_cancel_requested():
                self.get_logger().info(
                    "Eye Tracker Action Server preempted, stopping tracking."
                )
                await self._look_centre()
                goal_handle.canceled()
                image_sub.unregister()
                depth_sub.unregister()
                depth_camera_info_sub.unregister()
                self._done = True
                return EyeTrackerResult()

            await asyncio.sleep(0.25)

        goal_handle.succeed()
        image_sub.unregister()
        depth_sub.unregister()
        depth_camera_info_sub.unregister()
        return EyeTrackerResult()


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)

    # Create the node
    eye_tracker = EyeTracker()

    # This allows the action server to handle concurrent goals
    executor = MultiThreadedExecutor()
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
