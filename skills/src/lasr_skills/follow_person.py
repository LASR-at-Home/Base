import math
import rclpy
from rclpy.time import Time
import time
import traceback

import yasmin
from yasmin import State, Blackboard, StateMachine, Concurrence
import yasmin_ros
from yasmin_viewer import YasminViewerPub

import tf2_ros
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import (
    PointStamped,
    PoseWithCovarianceStamped,
    Pose,
    PoseStamped,
    PolygonStamped,
    Polygon,
    Point32,
)
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from shapely.geometry import Polygon as ShapelyPolygon

from lasr_skills import (
    Detect3DInArea,
    Wait,
    Say,
    GoToLocation,
    WaitForPersonInArea,
    AskAndListen,
)

from nav2_simple_commander.robot_navigator import BasicNavigator

# -----------------------------------------------
"""
    NAVIGATION LOGIC

"""


class WaitForNavGoal(State):
    """
    Acts as a safe idle buffer. Waits until the tracker provides a NEW goal
    and explicitly removes the stop request before allowing navigation to proceed.
    """

    def __init__(self):
        super().__init__(outcomes=["start_navigating", "failed"])
        self.node = yasmin_ros.logger_node
        self.last_goal = None

    def execute(self, blackboard: Blackboard):

        while not self.is_canceled() and rclpy.ok():
            stop_requested = blackboard.get("stop_robot_requested", False)
            current_goal = blackboard.get("location")

            # Only proceed if we aren't told to stop, AND the goal is actually new
            if (
                not stop_requested
                and current_goal is not None
                and current_goal != self.last_goal
            ):
                self.last_goal = current_goal
                return "start_navigating"

            try:
                time.sleep(0.5)
            except Exception:
                break

        return "failed"


class Navigator(StateMachine):
    """
    The concurrent navigation loop.
    Bounces safely between waiting for clearance and driving.
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])

        self.add_state(
            "WAIT_FOR_COMMAND",
            WaitForNavGoal(),
            transitions={"start_navigating": "DRIVE_TO_GOAL", "failed": "failed"},
        )

        self.add_state(
            "DRIVE_TO_GOAL",
            GoToLocation(),
            transitions={
                "succeeded": "WAIT_FOR_COMMAND",  # Arrived naturally? Wait for next command.
                "failed": "WAIT_FOR_COMMAND",  # Canceled by dynamic preemption? Loop back and check.
            },
        )


# -----------------------------------------------
"""
    TRACKER LOGIC

"""


class UpdateDetectionPolygon(State):
    """
    Transforms baselink coordinates into new map polygon after the robot moves.
    Writes: blackboard["polygon"]
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_output_key("polygon")

        self.node = yasmin_ros.logger_node

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        self.base_footprint_polygon = [
            [3.5, 1.5],
            [3.5, -1.5],
            [0.5, -0.5],
            [0.5, 0.5],
        ]

        self.debug_pub = self.node.create_publisher(
            PolygonStamped, "/person_follow/debug/polygon", 10
        )

    def execute(self, blackboard):

        try:
            transform = self.tf_buffer.lookup_transform(
                "map",  # Target frame
                "base_footprint",  # Source frame
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=5.0),
            )

            debug_polygon = PolygonStamped()
            debug_polygon.header.frame_id = "map"
            debug_polygon.header.stamp = Time().to_msg()

            transformed_polygon = []
            for pt in self.base_footprint_polygon:
                point_stamped = PointStamped()
                point_stamped.header.frame_id = "base_footprint"
                point_stamped.header.stamp = Time().to_msg()
                point_stamped.point.x = pt[0]
                point_stamped.point.y = pt[1]
                point_stamped.point.z = 0.0

                # Multiply the point by the transform matrix to get map coordinates
                mapped_point = do_transform_point(point_stamped, transform)
                transformed_polygon.append([mapped_point.point.x, mapped_point.point.y])

                p = Point32(x=mapped_point.point.x, y=mapped_point.point.y, z=0.0)
                debug_polygon.polygon.points.append(p)

            blackboard["polygon"] = ShapelyPolygon(transformed_polygon).buffer(0.05)

            self.debug_pub.publish(debug_polygon)
            yasmin.YASMIN_LOG_INFO("1. UPDATED POLYGON")
            return "succeeded"
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"TF transform failed: {e}")
            return "failed"


class EvaluateDetections(State):
    """
    Evaluates people detected in the polygon frame.
    Handles data association matching, stationary counting, and updating Nav2 blackboard targets.
    """

    def __init__(self, safe_distance=1.5):
        # Outcomes mapping perfectly back to your TrackPerson state machine
        super().__init__(
            outcomes=["updated", "paused", "person_stationary", "person_lost"]
        )

        # Pulls detections from the blackboard populated by Detect3DInArea
        self.add_input_key("detections_3d")
        self.add_input_key(
            "p_old"
        )  # if using getpersonpoint after wait for person in area pass and remap

        self.add_output_key("location")

        self.node = yasmin_ros.logger_node
        self.safe_distance = safe_distance

        # Internal Loop Memory Tracking
        self.stationary_count = 0
        self.p_old = None  # Stores the last known (x, y) map coordinate of the host
        self.blacklist = []
        # TODO:  Add later as an improvement but assume closest person is the correct one

        # Setup AMCL Pose Subscriber
        self.current_robot_point = None
        self.robot_pose_sub = self.node.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.robot_point_cb,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )
        yasmin.YASMIN_LOG_INFO(
            "EvaluateDetections initialized. Listening to /amcl_pose..."
        )

    def create_goal_pose(
        self, rx: float, ry: float, tx: float, ty: float, offset: bool = False
    ) -> Pose:
        """
        Unified goal constructor for both active pursuit and lost-target hunting.

        :param rx, ry: Current coordinates of the robot base.
        :param tx, ty: Coordinates of the target (either current or historical).
        :param offset: If True, offsets the goal by self.safe_distance. If False, targets point exactly.
        """
        # Calculate directional delta vectors
        dx = tx - rx
        dy = ty - ry
        distance = math.sqrt(dx**2 + dy**2)

        # Goal Point
        goal_x = tx
        goal_y = ty
        # If moving to a person stop before reaching thier point.
        if offset and distance > self.safe_distance:
            ratio = (
                distance - self.safe_distance
            ) / distance  # Distance to maintain safe distance
            goal_x = rx + (dx * ratio)
            goal_y = ry + (dy * ratio)

        # Calculate 2D planar heading angle (yaw) so the robot faces the target point
        theta = math.atan2(dy, dx)

        # Build and populate the standard ROS 2 Pose message
        goal_pose = Pose()
        goal_pose.position.x = goal_x
        goal_pose.position.y = goal_y
        goal_pose.position.z = 0.0

        goal_pose.orientation.x = 0.0
        goal_pose.orientation.y = 0.0
        goal_pose.orientation.z = math.sin(theta / 2.0)
        goal_pose.orientation.w = math.cos(theta / 2.0)

        return goal_pose

    def robot_point_cb(self, msg: PoseWithCovarianceStamped):
        """Stores the raw underlying Pose data on message arrival"""
        self.current_robot_point = msg.pose.pose.position

    def calc_distance_between_points(self, pointOne, pointTwo):
        """Calculates Eculidian distance between 2 given point"""
        return math.sqrt(
            (float(pointOne.x) - float(pointTwo.x)) ** 2
            + (float(pointOne.y) - float(pointTwo.y)) ** 2
        )

    def get_closest_person(self, detections):
        """Iterates through points and finds best person to go to."""
        return detections[0].point  # TODO: temp
        # I believe the first point is the closest one but double check
        # For each point check if in blacklist, if not choose point closest to p_old.

    def execute(self, blackboard: Blackboard):
        if self.current_robot_point is None:
            self.node.get_logger().warn("Robot pose not available yet, waiting...")
            return "paused" # or time.sleep(0.1) and continue in a loop

        if "p_old" in blackboard.keys() and self.p_old is None:
            self.p_old = blackboard["p_old"]

        if self.p_old is None:
            return "paused"

        detections = blackboard[
            "detections_3d"
        ]  # ros_ws/src/Base/common/vision/lasr_vision_interfaces/msg/Detection3D.msg

        if len(detections) == 0:

            # Threshold where it is worth moving
            threshold = 0.6
            if (
                self.calc_distance_between_points(self.current_robot_point, self.p_old)
                > threshold
            ):
                blackboard["location"] = self.create_goal_pose(
                    self.current_robot_point.x,
                    self.current_robot_point.y,
                    self.p_old.x,
                    self.p_old.y,
                    offset=False,
                )
                blackboard["stop_robot_requested"] = False
                self.stationary_count = 0
                self.node.get_logger().warn("NO PERSON BUT NAV GOAL AVAILABLE")
                return "updated"
            else:
                self.node.get_logger().warn("NO POSE OR PERSON FOUND")
                return "person_lost"

        # handling people still in polygon
        if len(detections) > 1:
            personPoint = self.get_closest_person(
                detections
            )  # Get closest person to p_old
            self.node.get_logger().warn("MORE THAN ONE PERSON FOUND")
            if personPoint == None:  # If all detected people are 'blacklisted'
                return "person_lost"
        else:
            self.node.get_logger().warn("ONE PERSON FOUND")
            personPoint = detections[0].point

        # handle person
        if (
            self.calc_distance_between_points(personPoint, self.current_robot_point)
            > self.safe_distance
        ):
            blackboard["location"] = self.create_goal_pose(
                self.current_robot_point.x,
                self.current_robot_point.y,
                personPoint.x,
                personPoint.y,
                offset=True,
            )
            blackboard["stop_robot_requested"] = False
            self.p_old = personPoint
            self.stationary_count = 0
            self.node.get_logger().warn("PERSON FOUND AND SETTING GOAL")
            return "updated"
        else:  # If person is too close or in same location
            blackboard["stop_robot_requested"] = True
            self.p_old = personPoint
            self.stationary_count += 1
            self.node.get_logger().warn(f"PERSON FOUND BUT STATIONARY: {self.stationary_count}/3")
            if self.stationary_count >= 3:
                self.node.get_logger().warn(f"PERSON STATIONARY")
                return "person_stationary"
            return "paused"


class TrackPerson(StateMachine):
    def __init__(self):
        # Outcomes align perfectly with your main locate_and_follow_host.py plan
        super().__init__(outcomes=["person_stationary", "person_lost", "failed"])

        # 1. Update the Map Area
        self.add_state(
            "UPDATE_POLYGON",
            UpdateDetectionPolygon(),
            transitions={
                "succeeded": "DETECT_3D",
                "failed": "failed",
            },
        )

        # 2. Scan the Area
        self.add_state(
            "DETECT_3D",
            Detect3DInArea(filter=["person"]),
            transitions={
                "succeeded": "EVALUATE_DETECTIONS",
                "failed": "EVALUATE_DETECTIONS",  # Go to evaluate anyway so the timeout logic can handle the empty list
            },
        )

        # 3. Process Math & Blackboard Updates
        self.add_state(
            "EVALUATE_DETECTIONS",
            EvaluateDetections(safe_distance=1.5),
            transitions={
                "updated": "WAIT_TICK",  # Goal changed, pause briefly
                "paused": "WAIT_TICK",  # Too close, pause briefly
                "person_stationary": "person_stationary",  # Breakout: Reached destination
                "person_lost": "person_lost",  # Breakout: Host vanished
            },
        )

        # 4. Short Loop Buffer
        self.add_state(
            "WAIT_TICK",
            Wait(
                1
            ),  # 0.5s is usually perfect for fluid tracking without overwhelming CPU
            transitions={"succeeded": "UPDATE_POLYGON", "failed": "UPDATE_POLYGON"},
        )


# -----------------------------------------------
"""
    Overall Following Logic
"""


class FollowPerson(StateMachine):
    def __init__(self):
        # Outcomes align perfectly with your main locate_and_follow_host.py plan
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)

        self.add_state(
            "UPDATE_POLYGON",
            UpdateDetectionPolygon(),
            transitions={
                "succeeded": "WAIT_FOR_HOST",
                "failed": "failed",
            },
        )
        self.add_state(
            "WAIT_FOR_HOST",
            WaitForPersonInArea(),  # Empty to use blackboard polygon
            transitions={
                "succeeded": "GET_PERSON_POINT",  # Host is infront of the robot
                "failed": "failed",  # Still waiting on host
            },
        )
        self.add_state(
            "GET_PERSON_POINT",
            yasmin.CbState(
                outcomes=["succeeded", "failed"], callback=self.get_person_point
            ),
            transitions={"succeeded": "SAY_FOLLOW", "failed": "WAIT_FOR_HOST"},
        )

        self.add_state(
            "SAY_FOLLOW",
            Say(text="I will now follow you. "),
            transitions={
                "succeeded": "succeeded",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        # # TRACK-NAV concur goes here
        # self.add_state(
        #     "TRACK_AND_NAVIGATE",
        #     Concurrence(
        #         states={
        #             "tracker": TrackPerson(),
        #             "navigator": Navigator(),
        #         },
        #         default_outcome="failed",
        #         outcome_map={
        #             "person_stationary": {
        #                 "tracker": "person_stationary",
        #             },
        #             "person_lost": {
        #                 "tracker": "person_lost",
        #             },
        #             "failed": {
        #                 "tracker": "failed",
        #                 "navigator": "failed",
        #             },
        #         },
        #     ),
        #     transitions={
        #         "person_stationary": "succeeded",
        #         "person_lost": "succeeded",
        #         "failed": "failed",
        #     },
        # )

        # # LOST_RECOVERY: Lose Person recovery (HEAD_TOUR + DETECT) if found person, approach and ask (if they are not the host add thier positon to a blacklist)

        # # Stationay Person
        # self.add_state(
        #     "ASK_IF_ARRIVED",
        #     AskAndListen(
        #         tts_phrase="Say YES if we have arrived. NO if we have not.",
        #     ),
        #     transitions={
        #         "succeeded": "succeeded",  # Update to HANDLE_RESPONSE
        #         "failed": "ASK_IF_ARRIVED",
        #     },
        #     remappings={"transcribed_speech": "guest_transcription"},
        # )

        # Callback which parses the resposne and returns "succeeded" or "SAY_FOLLOW"

    def get_person_point(
        self, blackboard
    ):  # will probably throw an error related to blackboard
        try:
            yasmin.YASMIN_LOG_INFO(f"ENTERED PERSON POINT CALLBACK WITH DETECTIONS: {blackboard['detections_3d']}")
            if not blackboard["detections_3d"]:
                return "failed"
            # Assuming the first detection is the point of interest
            blackboard["p_old"] = blackboard["detections_3d"][0].point
            return "succeeded"
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"The following error occured: {e}")
            return "failed"


def main():
    rclpy.init()
    yasmin_ros.set_ros_loggers()

    node = yasmin_ros.logger_node
    try:
        sm = TrackPerson()
        #sm = TrackPerson()
        bb = Blackboard()
        bb["z_sweep_min"] = -10
        bb["z_sweep_max"] = 50

        YasminViewerPub(sm, "Follow_Person")

        outcome = sm(bb)

        yasmin.YASMIN_LOG_INFO(outcome)
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(f"Exception in execution: {e}")
    finally:
        if node:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()



if __name__ == "__main__":
    main()
