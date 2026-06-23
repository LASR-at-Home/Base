import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
import message_filters
from threading import RLock, Event
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
    Vector3,
)
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

import time

class WaitForFuture():
    def __init__(self):
        self.event = Event()
        self.lock = RLock()
        self.response = None
        self.result = None
        self.status = None
        self.handle = None

    def set(self):
        self.event.clear()
    
    def handle_goal(self, future):
        with self.lock:
            self.handle = future.result()
            get_result_future = self.handle.get_result_async()
            get_result_future.add_done_callback(self.handle_result)

    def handle_result(self, future):
        self.result = future.result().result
        self.status = future.result().status
        self.event.set()

    def handle_resp(self, future):
        self.response = future.result()
        self.event.set()

        

class EyeTracker(Node):
    def __init__(self, max_eye_distance: float = 1.5):
        super().__init__("eye_tracker_action_server")

        # Humble deadlock avoidance: callbacks that make blocking service/action calls
        # must not share one mutually-exclusive group with their done-callbacks.

        self._done: bool = False
        self._eyes: Optional[Point] = None
        self._robot_point: Optional[Point] = None
        self._max_eye_distance: float = max_eye_distance
        self._move_up_count: int = 0
        self._max_move_up_count: int = 2

        self._action_cb_group = ReentrantCallbackGroup()
        self._work_cb_group = ReentrantCallbackGroup()

        self.camera_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        amcl_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._robot_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._robot_pose_callback,
            qos_profile=amcl_qos,
            callback_group=self._work_cb_group
        )
        
        self._yolo_keypoint_client = self.create_client(
            YoloPoseDetection3D,
            "/yolo/detect3d_pose",
            callback_group=self._work_cb_group
        )

        self._head_state_client = self.create_client(
            QueryTrajectoryState,
            "/head_controller/query_state",
            callback_group=self._work_cb_group
        )

        self._head_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/head_controller/follow_joint_trajectory",
            callback_group=self._work_cb_group
        )

        self._head_point_action_client = ActionClient(
            self,
            PointHead,
            "/head_controller/point_head_action",
            callback_group=self._work_cb_group
        )
        
        while not self._head_point_action_client.wait_for_server(timeout_sec=1.0) or not self._head_action_client.wait_for_server(timeout_sec=1.0) or not self._yolo_keypoint_client.wait_for_service(timeout_sec=1.0) or not self._head_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for point head action server, and head action client and yolo to all be ready...")

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
        request = QueryTrajectoryState.Request()
        request.time = self.get_clock().now().to_msg()

        wait = WaitForFuture()
        wait.set()

        future = self._head_state_client.call_async(request)
        future.add_done_callback(wait.handle_resp)
        self.get_logger().warn('Waiting for response from get head join values')
        while not wait.event.wait():
            pass
            
        if len(wait.response.position) < 2:
            self.get_logger().warn("Head state response was empty or invalid.")
            return None
        return (wait.response.position[0], wait.response.position[1])

    def _look_centre(self) -> None:
        """Moves the head to look at the centre position."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]  # Look Center
        point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()
        goal.trajectory.points.append(point)

        wait = WaitForFuture()
        wait.set()

        send_goal_future = self._head_action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(wait.handle_goal)
        self.get_logger().info('Waiting for response from look centre')
        while not wait.event.wait(0.5):
            self.get_logger().warn("Centre head goal was not accepted.")
            break

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
        point.time_from_start = rclpy.duration.Duration(seconds=2.0).to_msg()
        goal.trajectory.points.append(point)
        
        wait = WaitForFuture()
        wait.set()

        send_goal_future = self._head_action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(wait.handle_goal)
        self.get_logger().info('Waiting for move head up to be finished')
        while not wait.event.wait(0.5):
            self.get_logger().warn('Timed out for head movement, assuming finished')
            break

        self.get_logger().info(str(self._move_up_count))
        self.get_logger().info(str(self._max_move_up_count))
        # self.get_logger().info(self._move_up_count)
        self._move_up_count += 1

    def detect_cb(self, image: Image, depth_image: Image):
            """Callback for detection from synced messages."""
            req = YoloPoseDetection3D.Request(
                image_raw=image,
                depth_image=depth_image,
                depth_camera_info=self.depth_camera_info_cache.getLast(),
                model="yolo11n-pose.pt",
                confidence=0.5,
                target_frame="map",
            )
            
            wait = WaitForFuture()
            wait.set()
            
            future = self._yolo_keypoint_client.call_async(req)
            future.add_done_callback(wait.handle_resp)
            # self.get_logger().info('Waiting for yolo response in detect cb')
            while not wait.event.wait():
                pass

            detected_keypoints = wait.response.detections
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

    def _execute_callback(self, goal_handle):
        """Execute the eye tracking goal."""
        self.get_logger().info("Beginning eye tracking...")

        goal = goal_handle.request
        feedback_msg = EyeTrackerAction.Feedback()
        feedback_msg.running = False

        while self._robot_point is None:
            self.get_logger().warn('Waiting for robot pose')

        # First, look to person_point
        if goal.person_point is None:
            self.get_logger().error("No person point provided in goal.")
            goal_handle.abort()
            return EyeTrackerAction.Result()
        
        self.image_sub = message_filters.Subscriber(
            self, Image, "/head_front_camera/rgb/image_raw", self.camera_qos
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, "/head_front_camera/depth/image_raw", self.camera_qos
        )
        self.depth_camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, "/head_front_camera/depth/camera_info", self.camera_qos
        )
        self.depth_camera_info_cache = message_filters.Cache(self.depth_camera_info_sub)
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub], 10, 0.1
        )

        self.subs = [self.image_sub, self.depth_sub, self.depth_camera_info_sub]

        wait = WaitForFuture()
        wait.set()

        g = PointHead.Goal(
            pointing_frame="head_2_link",
            pointing_axis=Vector3(x=1.0, y=0.0, z=0.0),
            max_velocity=2.0,
            target=PointStamped(
                header=Header(frame_id="map"),
                point=goal.person_point,
            ),
        )

        # Send point head goal and wait
        send_goal_future = self._head_point_action_client.send_goal_async(g)
        send_goal_future.add_done_callback(wait.handle_goal)
        
        while not wait.event.wait(0.5):
            self.get_logger().warn("Timed out waiting for head controller to return a goal result, assuming it executed correctly")
            break
        
        self.ts.registerCallback(self.detect_cb)

        self._done = False
        feedback_msg.running = True
        while rclpy.ok() and not self._done:
            goal_handle.publish_feedback(feedback_msg)
            if self._eyes is None:
                current_head_position = self._get_head_join_values()
                if current_head_position is None:
                    continue
                # Move the head up by a small delta
                self._move_head_up(current_head_position, y_delta=0.25)
            else:
                g = PointHead.Goal(
                    pointing_frame="head_2_link",
                    pointing_axis=Vector3(x=1.0, y=0.0, z=0.0),
                    max_velocity=1.0,
                    target=PointStamped(
                        header=Header(frame_id="map"),
                        point=self._eyes,
                    ),
                )
                
                wait = WaitForFuture()
                wait.set()
                
                send_goal_future = self._head_point_action_client.send_goal_async(g)
                send_goal_future.add_done_callback(wait.handle_goal)
                self.get_logger().info('Waiting point head action result')
                while wait.event.wait(0.5):
                    self.get_logger().warn('Timed out for point head action, assuming it finished')
                    break

            if goal_handle.is_cancel_requested:
                self.get_logger().info(
                    "Eye Tracker Action Server canceled, stopping tracking."
                )
                self._look_centre()
                for sub in self.subs:
                    self.destroy_subscription(sub.sub)
                self._done = True
                goal_handle.canceled()

                self.get_logger().info('Canceled EYE TRACKER')

                return EyeTrackerAction.Result()
            
            time.sleep(0.25)

        goal_handle.succeed()
        return EyeTrackerAction.Result()


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
