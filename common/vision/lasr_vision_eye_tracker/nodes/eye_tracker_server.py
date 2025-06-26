#!/usr/bin/env python3
import rospy
import actionlib
import message_filters

from typing import Tuple, Optional

from lasr_vision_msgs.msg import (
    EyeTrackerGoal,
    EyeTrackerResult,
    EyeTrackerAction,
)
from lasr_vision_msgs.srv import (
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
    PoseStamped,
    PoseWithCovarianceStamped,
)
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

# Needs to be an action server, so it can be cancelled
# when we want tracking to stop.

# Request:
#   - person_point: PointStamped


# Look to person
# If can't see eyes, look up, repeat
# Look to midpoint of two eyes
# If midpoint as changed by some delta, look to midpoint again
# Maybe handle case where eyes move out of frame?


class EyeTracker:

    _action_server: actionlib.SimpleActionServer
    _done: bool
    _head_action_client: actionlib.SimpleActionClient
    _eyes: Optional[Point] = None
    _head_point_action_client: actionlib.SimpleActionClient
    _robot_pose_sub = rospy.Subscriber
    _yolo_keypoint_service: rospy.ServiceProxy
    _robot_point: Optional[Point] = None
    _max_eye_distance: float = 0.5

    def __init__(self, max_eye_distance: float = 0.5):

        self._action_server = actionlib.SimpleActionServer(
            "/lasr_vision_eye_tracker/track_eyes",
            EyeTrackerAction,
            self.execute,
            auto_start=False,
        )
        self._action_server.start()
        self._done = False
        self._eyes = None
        self._robot_point = None
        self._max_eye_distance = max_eye_distance

        self._robot_pose_sub = rospy.Subscriber(
            "/robot_pose",
            PoseWithCovarianceStamped,
            self._robot_pose_callback,
        )

        self._yolo_keypoint_service = rospy.ServiceProxy(
            "/yolo/detect3d_pose",
            YoloPoseDetection3D,
        )
        self._yolo_keypoint_service.wait_for_service()

        self._head_action_client = actionlib.SimpleActionClient(
            "/head_controller/follow_joint_trajectory", FollowJointTrajectoryAction
        )
        self._head_action_client.wait_for_server()

        self._head_point_action_client = actionlib.SimpleActionClient(
            "/head_controller/point_head_action", PointHeadAction
        )
        self._head_point_action_client.wait_for_server()

        rospy.loginfo("Eye Tracker Action Server started.")

    def _robot_pose_callback(self, msg: PoseStamped) -> None:
        """Callback for the robot pose subscriber.

        Updates the robot point based on the received pose.
        """
        self._robot_point = msg.pose.pose.position
        # rospy.loginfo(f"Robot pose updated: {self._robot_point}")

    def _get_head_join_values(self) -> Optional[Tuple[float, float]]:
        """Returns the x,y position of the head joints.

        Returns:
            Optional[Tuple[float, float]].
        """

        try:
            rospy.wait_for_service("/head_controller/query_state")
            joint_srv = rospy.ServiceProxy(
                "/head_controller/query_state", QueryTrajectoryState
            )
            resp = joint_srv(rospy.Time.now())
            return resp.position
        except rospy.ServiceException:
            rospy.loginfo("service call failed")
            return None

    def _move_head_up(
        self, current_head_position: Tuple[float, float], y_delta: float = 0.25
    ) -> None:
        """Moves the head up by a certain amount.

        Args:
            current_head_position (Tuple[float, float]): The current head joint values.
        """

        rospy.loginfo(
            f"Moving head up from position {current_head_position} to {current_head_position[1] + y_delta}"
        )
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [current_head_position[0], current_head_position[1] + y_delta]
        point.time_from_start = rospy.Duration(1)
        goal.trajectory.points.append(point)

        result = self._head_action_client.send_goal_and_wait(goal)
        rospy.loginfo(f"Head moved with result {result}")

    def execute(self, goal: EyeTrackerGoal):
        rospy.loginfo("Beginning eye tracking...")
        self._robot_point = rospy.wait_for_message(
            "/robot_pose", PoseWithCovarianceStamped, timeout=5.0
        ).pose.pose.position
        rospy.loginfo(f"Robot point set to: {self._robot_point}")
        # First, look to person_point
        if goal.person_point is None:
            rospy.logerr("No person point provided in goal.")
            self._action_server.set_aborted(
                result=EyeTrackerResult(), text="No person point provided."
            )
            return
        rospy.loginfo(f"Looking to person point: {goal.person_point}")
        g = PointHeadGoal(
            pointing_frame="head_2_link",
            pointing_axis=Point(1.0, 0.0, 0.0),
            max_velocity=1.0,
            target=PointStamped(
                header=Header(frame_id="map"),
                point=goal.person_point,
            ),
        )
        self._head_point_action_client.send_goal_and_wait(g)

        def detect_cb(image: Image, depth_image: Image, depth_camera_info: CameraInfo):
            req = YoloPoseDetection3DRequest(
                image_raw=image,
                depth_image=depth_image,
                depth_camera_info=depth_camera_info,
                model="yolo11n-pose.pt",
                confidence=0.5,
                target_frame="map",
            )
            response = self._yolo_keypoint_service(req)
            detected_keypoints = response.detections
            rospy.loginfo(f"Detected keypoints: {detected_keypoints}")
            left_eye_point = None
            right_eye_point = None
            if not detected_keypoints:
                rospy.loginfo("No keypoints detected.")
                self._eyes = None
                return
            if not self._robot_point:
                rospy.logwarn("Robot point not set, cannot calculate eyes position.")
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

                        eye_midpoint = Point(midpoint_x, midpoint_y, midpoint_z)
                        rospy.loginfo(f"Eyes found at midpoint: {self._eyes}")
                    elif left_eye_point:
                        eye_midpoint = Point(
                            left_eye_point.x, left_eye_point.y, left_eye_point.z
                        )
                        rospy.loginfo(f"Only left eye found at: {self._eyes}")
                    elif right_eye_point:
                        eye_midpoint = Point(
                            right_eye_point.x, right_eye_point.y, right_eye_point.z
                        )
                        rospy.loginfo(f"Only right eye found at: {self._eyes}")
                if eye_midpoint is not None:
                    # Calculate the distance from the robot point to the eye midpoint
                    distance = (
                        (eye_midpoint.x - self._robot_point.x) ** 2
                        + (eye_midpoint.y - self._robot_point.y) ** 2
                        + (eye_midpoint.z - self._robot_point.z) ** 2
                    ) ** 0.5
                    rospy.loginfo(f"Distance to eye midpoint: {distance}")
                    if distance < closest_distance:
                        closest_distance = distance
                        closest_eye_midpoint = eye_midpoint
            if closest_eye_midpoint is not None:
                rospy.loginfo(f"Closest eye midpoint found at: {closest_eye_midpoint}")
                self._eyes = closest_eye_midpoint

        image_sub = message_filters.Subscriber("/xtion/rgb/image_raw", Image)
        depth_sub = message_filters.Subscriber(
            "/xtion/depth_registered/image_raw", Image
        )
        depth_camera_info_sub = message_filters.Subscriber(
            "/xtion/depth_registered/camera_info", CameraInfo
        )
        ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub, depth_camera_info_sub], 10, 0.1
        )
        ts.registerCallback(detect_cb)

        self._done = False
        while not self._done:
            if self._eyes is None:
                rospy.loginfo("No eyes detected, moving head up to try again.")
                current_head_position = self._get_head_join_values()
                if current_head_position is None:
                    rospy.logerr("Failed to get head joint values.")
                    continue
                # Move the head up by a small delta
                self._move_head_up(current_head_position, y_delta=0.25)
                rospy.sleep(0.5)  # Wait for the head to move
            else:
                rospy.loginfo(f"Tracking eyes at position: {self._eyes}")
                g = PointHeadGoal(
                    pointing_frame="head_2_link",
                    pointing_axis=Point(1.0, 0.0, 0.0),
                    max_velocity=1.0,
                    target=PointStamped(
                        header=Header(frame_id="map"),
                        point=self._eyes,
                    ),
                )
                self._head_point_action_client.send_goal_and_wait(g)

            # Check if the action has been preempted
            if self._action_server.is_preempt_requested():
                rospy.loginfo("Eye Tracker Action Server preempted, stopping tracking.")
                self._action_server.set_preempted()
                image_sub.unregister()
                depth_sub.unregister()
                depth_camera_info_sub.unregister()
                self._done = True
                return
            rospy.sleep(0.25)


if __name__ == "__main__":
    rospy.init_node("eye_tracker_action_server")
    server = EyeTracker()
    rospy.spin()
