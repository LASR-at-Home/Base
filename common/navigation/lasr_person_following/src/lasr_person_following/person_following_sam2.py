#!/usr/bin/env python3
import rospy
import math

from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    Pose,
    PoseStamped,
    Quaternion,
    PoseArray,
    Point,
)

from typing import Union, List, Tuple

from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import PointCloud2
import message_filters

import rosservice
import tf2_ros as tf
import tf2_geometry_msgs  # type: ignore
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from nav_msgs.srv import GetPlan
from play_motion_msgs.msg import PlayMotionAction
from play_motion_msgs.msg import PlayMotionGoal
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from lasr_vision_msgs.msg import Sam2PromptArrays as PromptArrays, Sam2BboxWithFlag as BboxWithFlag, Detection3DArray, Detection3D
from lasr_vision_msgs.srv import YoloDetection3D, YoloDetection3DRequest

from math import atan2
import numpy as np
from scipy.spatial.transform import Rotation as R

import actionlib
from control_msgs.msg import PointHeadAction, PointHeadGoal

from lasr_person_following.msg import (
    FollowAction,
    FollowGoal,
    FollowResult,
    FollowFeedback,
)

MAX_VISION_DIST: float = 5.0


class PersonFollower:
    # Parameters
    _start_following_radius: float
    _start_following_angle: float
    _new_goal_threshold: float
    _stopping_distance: float
    _vision_recovery_motions: List[str] = ["look_centre", "look_left", "look_right"]
    _vision_recovery_attempts: int = 3

    # Action clients
    _move_base_client: actionlib.SimpleActionClient

    # Services
    _make_plan: rospy.ServiceProxy

    # Tf
    _buffer: tf.Buffer
    _listener: tf.TransformListener

    # Publishers
    _person_trajectory_pub: rospy.Publisher

    # Waypoints
    _waypoints: List[PoseStamped]

    def __init__(
        self,
        start_following_radius: float = 2.0,
        start_following_angle: float = 45.0,
        new_goal_threshold: float = 2.0,
        stopping_distance: float = 1.0,
        static_speed: float = 0.0015,
        max_speed: float = 0.55,
    ):
        self._start_following_radius = start_following_radius
        self._start_following_angle = start_following_angle
        self._new_goal_threshold = new_goal_threshold
        self._stopping_distance = stopping_distance
        self._static_speed = static_speed
        self._max_speed = max_speed

        self._track_bbox = None
        self._track_id = None

        rospy.init_node("person_following_sam2")

        self.camera = rospy.get_param("~camera", "xtion")
        self.model_name = rospy.get_param("~model", "yolo11n-seg.pt")

        self.image_topic = f"/{self.camera}/rgb/image_raw"
        self.depth_topic = f"/{self.camera}/depth_registered/image_raw"
        self.depth_camera_info_topic = f"/{self.camera}/depth_registered/camera_info"

        self.image, self.depth_image, self.depth_camera_info = None, None, None
        image_sub = message_filters.Subscriber(self.image_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        depth_camera_info_sub = message_filters.Subscriber(
            self.depth_camera_info_topic, CameraInfo
        )
        ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub, depth_camera_info_sub], 10, 0.1
        )
        ts.registerCallback(self.image_callback)

        self.prompt_pub = rospy.Publisher("/sam2/prompt_arrays", PromptArrays, queue_size=1)
        self.track_flag_pub = rospy.Publisher("/sam2/track_flag", Bool, queue_size=1)
        self.detection3d_sub = rospy.Subscriber(
                "/sam2/detections_3d", Detection3DArray, self.detection3d_callback, queue_size=1
            )
        self.trajectory_marker_pub = rospy.Publisher(
            "/person_trajectory_markers", MarkerArray, queue_size=1
        )
        self.yolo = rospy.ServiceProxy("/yolo/detect3d", YoloDetection3D)
        self.yolo.wait_for_service()

        self._move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        if not self._move_base_client.wait_for_server(rospy.Duration.from_sec(10.0)):
            rospy.logwarn("Move base client not available")

        self._buffer = tf.Buffer(cache_time=rospy.Duration.from_sec(10.0))
        self._listener = tf.TransformListener(self._buffer)

        rospy.wait_for_service(
            "/move_base/make_plan", timeout=rospy.Duration.from_sec(10.0)
        )
        self._make_plan = rospy.ServiceProxy("/move_base/make_plan", GetPlan)

        self._person_trajectory_pub = rospy.Publisher(
            "/person_trajectory", PoseArray, queue_size=1, latch=True
        )

        self._should_stop = False

        self._waypoints = []
        self._last_detection_time = rospy.Time.now()
        self.newest_detection = None

        self._point_head_client = actionlib.SimpleActionClient(
            "/head_controller/point_head_action", PointHeadAction
        )
        if not self._point_head_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logwarn("Head pointing action server not available.")

        self._last_head_motion = "look_centre"
        self._head_motion_interval = rospy.Duration(2.0)
        self._last_head_motion_time = rospy.Time.now()
        self._look_centre_count = 0
        self._look_centre_point()

    def _look_at_point(self, target_point: Point, target_frame: str = "map"):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = target_frame
        goal.target.point = target_point

        goal.pointing_frame = "xtion_rgb_optical_frame"
        goal.pointing_axis.x = 0.0
        goal.pointing_axis.y = 0.0
        goal.pointing_axis.z = 1.0

        goal.min_duration = rospy.Duration(0.3)
        goal.max_velocity = 1.0

        self._point_head_client.send_goal(goal)

    def _enqueue_head_motion_sequence(self):
        now = rospy.Time.now()
        self._head_motion_sequence = [
            ("look_down_centre", now),
            ("look_centre", now + rospy.Duration(2.0)),
            ("look_target", now + rospy.Duration(2.0)),
            ("look_down_centre", now + rospy.Duration(2.0)),  # after 1s holding target
        ]
        self._head_motion_active = True

    def image_callback(self, image: Image, depth_image: Image, depth_camera_info: CameraInfo):
        self.image, self.depth_image, self.depth_camera_info = image, depth_image, depth_camera_info

    def _tf_pose(self, pose: PoseStamped, target_frame: str):
        trans = self._buffer.lookup_transform(
            target_frame, pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0)
        )
        return do_transform_pose(pose, trans)

    def _robot_pose_in_odom(self) -> Union[PoseStamped, None]:
        try:
            current_pose: PoseWithCovarianceStamped = rospy.wait_for_message(
                "/robot_pose",
                PoseWithCovarianceStamped,
                timeout=rospy.Duration.from_sec(2.0),
            )
        except rospy.ROSException:
            return None
        except AttributeError:
            return None

        current_pose_stamped = PoseStamped(
            pose=current_pose.pose.pose, header=current_pose.header
        )

        return self._tf_pose(current_pose_stamped, "odom")

    def begin_tracking(self) -> bool:
        """
        Chooses the closest person as the target
        """

        while True:
            if self.image and self.depth_image and self.depth_camera_info:
                break
        req = YoloDetection3DRequest(
            image_raw=self.image,
            depth_image=self.depth_image,
            depth_camera_info=self.depth_camera_info,
            model="yolo11n-pose.pt",  # not using poses for now
            confidence=0.5,
            target_frame="map",
        )
        response = self.yolo(req)
        print(response)
        detections: Detection3D = response.detected_objects

        detected_people = {
            "xywh": [],
            "point": [],
        }
        for detection in detections:
            detected_people["point"].append(detection.point)
            detected_people["xywh"].append(detection.xywh)

        # find the nearest person:
        robot_frame = "base_link"
        map_frame = "map"

        # find robot's position in map
        try:
            transform = self._buffer.lookup_transform(
                map_frame,
                robot_frame,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
        except tf.LookupException as e:
            rospy.logerr("TF lookup failed: %s", str(e))
            return

        min_dist = float("inf")
        nearest_index = -1

        for i, point in enumerate(detected_people["point"]):
            dx = point.x - robot_x
            dy = point.y - robot_y
            dist = math.hypot(dx, dy)

            if dist < min_dist:
                min_dist = dist
                nearest_index = i

        if nearest_index != -1:
            nearest_person_point = detected_people["point"][nearest_index]
            nearest_person_bbox = detected_people["xywh"][nearest_index]
            rospy.loginfo(f"Nearest person at distance {min_dist:.2f}m")
        else:
            return False

        self._track_bbox = nearest_person_bbox
        self._track_id = nearest_index

        # restart sam2 and prompt it
        bbox_list = []
        for i, bbox in enumerate(detected_people["xywh"]):
            bbox_msg = BboxWithFlag()
            bbox_msg.obj_id = i
            bbox_msg.reset = False  # not to reset here, will reset together
            bbox_msg.clear_old_points = True
            bbox_msg.xywh = bbox
            bbox_list.append(bbox_msg)
            rospy.loginfo(f"Prepared BBox for ID {i}")

        prompt_msg = PromptArrays()
        prompt_msg.bbox_array = bbox_list
        prompt_msg.point_array = []
        prompt_msg.reset = True  # full reset
        self.prompt_pub.publish(prompt_msg)
        rospy.loginfo(f"Published PromptArrays with {len(bbox_list)} BBoxes.")
        rospy.sleep(0.5)
        self.track_flag_pub.publish(Bool(data=True))
        rospy.sleep(0.1)
        self.track_flag_pub.publish(Bool(data=True))
        rospy.sleep(0.1)
        self.track_flag_pub.publish(Bool(data=True))
        rospy.sleep(0.1)
        rospy.loginfo(f"Published SAM2 traking commmand.")
        rospy.sleep(0.5)

        rospy.loginfo(f"Tracking person discovered with id {self._track_id}")
        return True

    def _euclidian_distance(self, p1: Pose, p2: Pose) -> float:
        return np.linalg.norm(
            np.array([p1.position.x, p1.position.y])
            - np.array([p2.position.x, p2.position.y])
        ).astype(float)

    def _quat_to_dir(self, q: Quaternion):
        x, y, z, w = q.x, q.y, q.z, q.w
        forward = np.array(
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)]
        )
        return forward

    def _angle_between_vectors(self, v1, v2):
        dot_product = np.dot(v1, v2)
        norms_product = np.linalg.norm(v1) * np.linalg.norm(v2)
        cos_theta = dot_product / norms_product
        cos_theta = np.clip(cos_theta, -1.0, 1.0)
        angle = np.arccos(cos_theta)
        return angle

    def _compute_face_quat(self, p1: Pose, p2: Pose) -> Quaternion:
        dx: float = p2.position.x - p1.position.x
        dy: float = p2.position.y - p1.position.y
        theta_deg = np.degrees(atan2(dy, dx))
        x, y, z, w = R.from_euler("z", theta_deg, degrees=True).as_quat()
        return Quaternion(x, y, z, w)

    def _cancel_goal(self) -> None:
        if self._move_base_client.get_state() in [
            GoalStatus.PENDING,
            GoalStatus.ACTIVE,
        ]:
            self._move_base_client.cancel_goal()
            self._move_base_client.wait_for_result()

    def _get_pose_on_path(
        self, start_pose: PoseStamped, goal_pose: PoseStamped, dist_to_goal: float
    ) -> Union[None, PoseStamped]:
        start: rospy.Time = rospy.Time.now()
        assert (
            start_pose.header.frame_id == goal_pose.header.frame_id
        ), "Start and goal poses must be in the same frame"

        chosen_pose: Union[None, PoseStamped] = None

        # make a plan from start to goal
        try:
            plan = self._make_plan(start_pose, goal_pose, self._stopping_distance).plan
        except rospy.ServiceException as e:
            rospy.loginfo(e)
            return chosen_pose

        # select a pose that is dist_to_goal away from the goal_pose
        for pose in reversed(plan.poses):
            if self._euclidian_distance(pose.pose, goal_pose.pose) >= dist_to_goal:
                chosen_pose = pose
                break
        end: rospy.Time = rospy.Time.now()

        rospy.logwarn(f"Time taken to find a plan: {end.secs - start.secs} seconds")

        return chosen_pose

    def _move_base(self, pose: PoseStamped, wait=False) -> MoveBaseGoal:
        goal: MoveBaseGoal = MoveBaseGoal()
        goal.target_pose = pose
        self._move_base_client.send_goal(goal)

        if wait:
            self._move_base_client.wait_for_result()

        return goal
    
    def _recover_track(self) -> bool:
        while not self.begin_tracking() and not rospy.is_shutdown():
            rospy.loginfo("Recovering track...")
            rospy.sleep(1)
        return True
    
    def detection3d_callback(self, msg: Detection3DArray):
        for detection in msg.detections:
            if int(detection.name) == self._track_id:
                self.newest_detection = detection
                self._last_detection_time = rospy.Time.now()
                break

    def _look_centre_point(self):
        point = Point(x=3.0, y=0.0, z=1.3)
        self._look_at_point(point, target_frame="base_link")

    def _look_down_centre_point(self):
        point = Point(x=3.0, y=0.0, z=0.0)
        self._look_at_point(point, target_frame="base_link")


    def follow(self) -> FollowResult:
        result = FollowResult()
        person_trajectory: PoseArray = PoseArray()
        person_trajectory.header.frame_id = "odom"
        prev_goal = None
        prev_pose = None
        track_vels: List[Tuple[float, float]] = []

        # Variables for velocity estimation
        self._prev_detection_pos = None  # (x, y)
        self._prev_detection_time = None  # rospy.Time

        rate = rospy.Rate(10)
        timeout_duration = rospy.Duration(5.0)

        while not rospy.is_shutdown():
            # 1. Timeout check: no detection for 5 seconds
            if (
                not hasattr(self, "_last_detection_time")
                or rospy.Time.now() - self._last_detection_time > timeout_duration
            ):
                rospy.loginfo("Lost track of person, recovering...")
                person_trajectory = PoseArray()
                self._recover_track()
                prev_pose = None
                self._prev_detection_pos = None
                self._prev_detection_time = None
                track_vels.clear()
                rate.sleep()
                continue

            # 2. Get newest_detection
            detection = getattr(self, "newest_detection", None)
            if detection is None:
                rospy.logwarn("No detection yet")
                rate.sleep()
                continue

            # 3. Create PoseStamped in "map"
            map_pose = PoseStamped()
            map_pose.header.frame_id = "map"
            map_pose.header.stamp = rospy.Time.now()  # Conservative estimate
            map_pose.pose.position = detection.point
            map_pose.pose.orientation.w = 1.0  # Identity orientation

            # 4. Transform to "odom"
            try:
                odom_pose = self._buffer.transform(
                    map_pose, "odom", rospy.Duration(0.5)
                )
            except (tf.LookupException, tf.ExtrapolationException) as e:
                rospy.logwarn(f"TF transform failed: {e}")
                rate.sleep()
                continue

            # 5. Add position to trajectory
            person_trajectory.poses.append(odom_pose.pose)

            # Publish as MarkerArray
            marker_array = MarkerArray()

            for idx, pose in enumerate(person_trajectory.poses):
                marker = Marker()
                marker.header.frame_id = "odom"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "person_trajectory"
                marker.id = idx
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose = pose
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
                marker_array.markers.append(marker)

            line_marker = Marker()
            line_marker.header.frame_id = "odom"
            line_marker.header.stamp = rospy.Time.now()
            line_marker.ns = "person_trajectory"
            line_marker.id = 9999
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.05
            line_marker.color.a = 1.0
            line_marker.color.r = 0.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
            line_marker.points = [pose.position for pose in person_trajectory.poses]
            marker_array.markers.append(line_marker)

            self.trajectory_marker_pub.publish(marker_array)

            # 6. Estimate velocity (x, y in odom)
            now = rospy.Time.now()
            curr_x = odom_pose.pose.position.x
            curr_y = odom_pose.pose.position.y
            vel_x, vel_y = 0.0, 0.0

            if self._prev_detection_pos is not None and self._prev_detection_time is not None:
                dt = (now - self._prev_detection_time).to_sec()
                if dt > 0:
                    dx = curr_x - self._prev_detection_pos[0]
                    dy = curr_y - self._prev_detection_pos[1]
                    vel_x = dx / dt
                    vel_y = dy / dt

            self._prev_detection_pos = (curr_x, curr_y)
            self._prev_detection_time = now

            # 7. Velocity tracking buffer
            track_vels.append((vel_x, vel_y))
            if len(track_vels) > 10:
                track_vels.pop(0)

            # --- Head motion control based on velocity ---
            avg_speed = np.mean([np.linalg.norm(vel) for vel in track_vels]) if track_vels else 0.0
            now_time = rospy.Time.now()

            def try_look_at_target():
                try:
                    if self.newest_detection is None:
                        return False
                    point = self.newest_detection.point
                    target_frame = "xtion_rgb_optical_frame"
                    self._buffer.lookup_transform(
                        target_frame,
                        "map",  # newest_detection.point
                        rospy.Time(0),
                        rospy.Duration(0.5)
                    )
                    self._look_at_point(point, target_frame="map")
                    return True
                except (tf.LookupException, tf.ExtrapolationException, tf.ConnectivityException):
                    return False

            if avg_speed < self._static_speed:
                if now_time - self._last_head_motion_time > self._head_motion_interval:
                    self._look_centre_point()
                    self._last_head_motion = "look_centre"
                    self._last_head_motion_time = now_time
                    self._look_centre_count = 0
            else:
                if now_time - self._last_head_motion_time > self._head_motion_interval:
                    if self._look_centre_count < 2:
                        self._look_centre_point()
                        self._last_head_motion = "look_centre"
                        self._look_centre_count += 1
                    else:
                        self._look_down_centre_point()
                        self._last_head_motion = "look_down_centre"
                        self._look_centre_count = 0
                    self._last_head_motion_time = now_time

            # 8. Goal logic
            if prev_pose is None:
                robot_pose = self._robot_pose_in_odom()
                goal_orientation = self._compute_face_quat(robot_pose.pose, odom_pose.pose)
                goal_pose = self._tf_pose(
                    PoseStamped(
                        pose=Pose(
                            position=odom_pose.pose.position,
                            orientation=goal_orientation,
                        ),
                        header=odom_pose.header,
                    ),
                    "map",
                )
                self._move_base(goal_pose)
                prev_goal = goal_pose
                prev_pose = odom_pose

            else:
                dist_to_prev = self._euclidian_distance(odom_pose.pose, prev_pose.pose)

                if dist_to_prev >= self._new_goal_threshold:
                    goal_orientation = self._compute_face_quat(robot_pose.pose, odom_pose.pose)
                    pose_with_orientation = Pose(
                        position=odom_pose.pose.position,
                        orientation=goal_orientation,
                    )
                    goal_pose = self._tf_pose(
                        PoseStamped(pose=pose_with_orientation, header=odom_pose.header),
                        "map",
                    )
                    self._move_base(goal_pose)
                    self._waypoints.append(goal_pose)
                    prev_goal = goal_pose
                    prev_pose = odom_pose

                # elif (
                #     self._move_base_client.get_state() in [GoalStatus.ABORTED]
                #     and prev_goal is not None
                # ):
                #     rospy.logwarn("Goal was aborted, retrying")
                #     self._move_base(prev_goal)

                # elif (
                #     (
                #         np.mean([np.linalg.norm(vel) for vel in track_vels])
                #         < self._static_speed
                #     )
                #     and len(track_vels) == 10
                # ):
                #     rospy.logwarn("Person has been static too long, stopping")
                #     self._cancel_goal()
                #     track_vels = []
                #     self._waypoints = []

                elif self._move_base_client.get_state() in [GoalStatus.SUCCEEDED]:
                    goal_orientation = self._compute_face_quat(robot_pose.pose, odom_pose.pose)
                    pose_with_orientation = Pose(
                        position=odom_pose.pose.position,
                        orientation=goal_orientation,
                    )
                    goal_pose = self._tf_pose(
                        PoseStamped(pose=pose_with_orientation, header=odom_pose.header),
                        "map",
                    )
                    self._move_base(goal_pose)
                    prev_goal = goal_pose
                    prev_pose = odom_pose

            rate.sleep()

        return result
    

if __name__ == "__main__":
    follower = PersonFollower()
    follower.follow()
    rospy.spin()
