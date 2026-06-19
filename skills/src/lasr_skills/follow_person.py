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

from std_msgs.msg import Header
from geometry_msgs.msg import (
    PointStamped,
    PoseWithCovarianceStamped,
    Pose,
    PolygonStamped,
    Point32,
)
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from shapely.geometry import Polygon as ShapelyPolygon

from lasr_skills import (
    Detect3DInArea,
    Wait,
    PlayMotion,
    Say,
    ContinuousGoToLocation,
    WaitForPersonInArea,
    LookToPoint,
    AskAndListen,
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
            [3.5, 1.25],         # Top Left
            [3.5, -1.25],        # Top Right
            [-0.2, -1.75],        # Bottom Right
            [-0.2, 1.75],         # Bottom Left
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
            return "succeeded"
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"TF transform failed: {e}")
            return "failed"


class EvaluateDetections(State):
    """
    Evaluates people detected in the polygon frame.
    Handles data association matching, stationary counting, and updating Nav2 blackboard targets.
    """

    def __init__(self, safe_distance=1, threshold=0.25, max_stationary=5):
        # Outcomes mapping perfectly back to your TrackPerson state machine
        super().__init__(
            outcomes=["updated", "paused", "person_stationary", "person_lost"]
        )

        # Pulls detections from the blackboard populated by Detect3DInArea
        self.add_input_key("detections_3d")
        self.add_input_key("last_known")  
        # if using getpersonpoint after wait for person in area pass and remap

        self.add_output_key("location")
        self.add_output_key("cancel_nav")

        self.node = yasmin_ros.logger_node
        self.safe_distance = safe_distance
        self.threshold = threshold
        self.max_stationary = max_stationary

        # Internal Loop Memory Tracking
        self.stationary_count = 0
        self.last_known = None  # Stores the last known (x, y) map coordinate of the person


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.current_robot_point = None

    def create_goal_pose(
        self, rx: float, ry: float, tx: float, ty: float, offset: bool = False
    ) -> Pose:
        # Calculate directional delta vectors
        dx = tx - rx
        dy = ty - ry
        distance = math.sqrt(dx**2 + dy**2)

        # Goal Point
        goal_x = tx
        goal_y = ty
        # If moving to a person stop before reaching thier point.
        if offset and distance > self.safe_distance:
            goal_x = tx - (dx / distance) * self.safe_distance
            goal_y = ty - (dy / distance) * self.safe_distance

        # Calculate 2D planar heading angle (yaw) so the robot faces the target point
        theta = math.atan2(dy, dx)

        # Build and populate the standard ROS 2 Pose message
        goal_pose = Pose()
        goal_pose.position.x = goal_x
        goal_pose.position.y = goal_y
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
        best_person = None
        closest_d = float('inf')

        for person in detections:
            distance = self.calc_distance_between_points(person.point, self.last_known)
            if distance < closest_d:
                closest_d = distance
                best_person = person.point
        return best_person


    def execute(self, blackboard: Blackboard):
        # retrive robot's location in map
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",  
                "base_footprint",  
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            self.current_robot_point = transform.transform.translation
        except Exception as e:
            self.node.get_logger().warn(f"Waiting for map-base_footprint TF: {e}")
            return "paused"

        # Handle blackboard data
        if "last_known" in blackboard.keys() and self.last_known != blackboard["last_known"]:
            self.last_known = blackboard["last_known"]

        detections = blackboard['detections_3d']
        if self.last_known is None:
            if len(detections) > 0:
                self.node.get_logger().info("First detection found. Initializing last_known.")
                self.last_known = detections[0].point
            else:
                self.node.get_logger().warn("Waiting for first person detection...")
                return "paused"
            
        if "cancel_nav" not in blackboard.keys():
            blackboard["cancel_nav"] = False
            
        # People found in frame
        if len(detections) > 0:
            personPoint = self.get_closest_person(detections)
            
            if len(detections) > 1:
                self.node.get_logger().warn("Multiple people in polygon. Tracking closest to last known.")
            else:
                self.node.get_logger().info("Target locked.")
 
            blackboard["person_lost"] = False
 
            distance_person_moved = self.calc_distance_between_points(personPoint, self.last_known)
            distance_robot_from_person = self.calc_distance_between_points(personPoint, self.current_robot_point)
 
            if distance_person_moved > self.threshold:
                # Person is actively moving — reset stationary counter and chase
                self.last_known = personPoint
                blackboard["last_known"] = personPoint
                self.stationary_count = 0
 
                if distance_robot_from_person > (self.safe_distance + 0.2):
                    blackboard["location"] = self.create_goal_pose(
                        self.current_robot_point.x,
                        self.current_robot_point.y,
                        self.last_known.x,
                        self.last_known.y,
                        offset=True,
                    )
                    blackboard["stop_robot_requested"] = False
                    self.node.get_logger().warn("PERSON FOUND: UPDATING NAV GOAL")
                    return "updated"
                else:
                    # Robot is already within safe_distance of the person
                    blackboard["stop_robot_requested"] = True
                    self.node.get_logger().warn("PERSON TOO CLOSE TO NAVIGATE")
                    return "paused"
            else:
                # Person has not moved significantly this tick
                self.last_known = personPoint
                blackboard["last_known"] = personPoint
 
                if distance_robot_from_person <= (self.safe_distance + 0.2):
                    # Robot is close and person is stationary → count up
                    self.stationary_count += 1
                    blackboard["stop_robot_requested"] = True
                    self.node.get_logger().warn(
                        f"PERSON STATIONARY: {self.stationary_count}/{self.max_stationary}"
                    )
                else:
                    # Person hasn't moved but robot hasn't caught up yet — keep following
                    self.stationary_count = 0
                    blackboard["location"] = self.create_goal_pose(
                        self.current_robot_point.x,
                        self.current_robot_point.y,
                        self.last_known.x,
                        self.last_known.y,
                        offset=True,
                    )
                    blackboard["stop_robot_requested"] = False
 
            if self.stationary_count >= self.max_stationary:
                self.node.get_logger().warn(
                    f"PERSON CONFIRMED STATIONARY AFTER {self.stationary_count} TICKS"
                )
                blackboard["cancel_nav"] = True
                return "person_stationary"
 
            return "paused"
 
        distance_old_from_robot = self.calc_distance_between_points(
            self.last_known, self.current_robot_point
        )
        if distance_old_from_robot > self.threshold + 0.3:
            # Robot hasn't reached the last known position yet — keep driving
            blackboard["location"] = self.create_goal_pose(
                self.current_robot_point.x,
                self.current_robot_point.y,
                self.last_known.x,
                self.last_known.y,
                offset=False,
            )
            blackboard["stop_robot_requested"] = False
            self.node.get_logger().warn("NO PERSON: MOVING TO LAST KNOWN POSITION.")
            return "updated"
        else:
            self.node.get_logger().warn("NO PERSON FOUND. ALREADY AT LAST KNOWN.")
            blackboard["stop_robot_requested"] = True
            return "person_lost"

class InitialRecovery(StateMachine):
    class ScanForPerson(StateMachine):
        def __init__(self, direction: str="center"):
            super().__init__(outcomes=["succeeded", "failed"])
            self.add_output_key("last_known")
            
            self.add_state(
                "PLAYMOTION",
                PlayMotion(f"look_{direction}"),
                transitions={
                    "succeeded": "DETECT_3D",
                    "aborted": "failed",
                    "canceled": "failed",
                },
            )
            self.add_state(
                "DETECT_3D",
                Detect3DInArea(filter=["person"]),
                transitions={
                    "succeeded": "GET_PERSON_POINT",
                    "failed": "failed",
                },
            )
            self.add_state(
                "GET_PERSON_POINT",
                GetPersonPoint(),
                transitions={
                    "succeeded": "succeeded", 
                    "failed": "failed"
                },
        )
    
    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_output_key("last_known")
        
        self.add_state(
            "SAY_RECOVERING",
            Say(text="I can't see you."),
            transitions={
                "succeeded": "DETECT_3D_CENTER",
                "aborted": "DETECT_3D_CENTER",
                "canceled": "DETECT_3D_CENTER",
            },
        )

        self.add_state(
            "DETECT_3D_CENTER",
            self.ScanForPerson("center"),
            transitions={
                "succeeded": "succeeded",
                "failed": "DETECT_3D_LEFT",
            },
        )

        self.add_state(
            "DETECT_3D_LEFT",
            self.ScanForPerson("left"),
            transitions={
                "succeeded": "succeeded",
                "failed": "DETECT_3D_RIGHT",
            },
        )

        self.add_state(
            "DETECT_3D_RIGHT",
            self.ScanForPerson("right"),
            transitions={
                "succeeded": "succeeded",
                "failed": "failed",
            },
        )


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
        self.add_state(
            "DETECT_3D",
            Detect3DInArea(filter=["person"]),
            transitions={
                "succeeded": "EVALUATE_DETECTIONS",
                "failed": "failed",
            },
        )

        # 3. Process Math & Blackboard Updates
        self.add_state(
            "EVALUATE_DETECTIONS",
            EvaluateDetections(),
            transitions={
                "updated": "WAIT_TICK",  # Goal changed, pause briefly
                "paused": "WAIT_TICK",  # Too close, pause briefly
                "person_stationary": "person_stationary",  # Breakout: Reached destination
                "person_lost": "BASIC_RECOVERY",  # Breakout: Host vanished
            },
        )

        self.add_state(
            "LOOK",
            LookToPoint(),
            transitions={
                "succeeded": "succeeded",
                "aborted": "failed",
                "canceled": "failed",
            },
            remappings={"pointstamped": "last_known_stamped"}
        )


        # 4. Short Loop Buffer
        self.add_state(
            "WAIT_TICK",
            Wait(0.2),  
            transitions={"succeeded": "UPDATE_POLYGON", "failed": "failed"},
        )

        self.add_state(
            "BASIC_RECOVERY",
            InitialRecovery(),
            transitions={
                "succeeded": "EVALUATE_DETECTIONS", 
                "failed": "person_lost"
            },
        )


# -----------------------------------------------
"""
    Overall Following Logic
"""

class GetPersonPoint(State):
    def __init__(self):
        # Outcomes mapping perfectly back to your TrackPerson state machine
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("detections_3d")
        self.add_output_key("last_known") 
        self.add_output_key("last_known_stamped")  

    def execute(self, blackboard):
        
        try:
            if not blackboard["detections_3d"]:
                return "failed"
            # Assuming the first detection is the point of interest
            last_known = blackboard["detections_3d"][0].point
            blackboard["last_known"] = last_known

            blackboard["last_known_stamped"] = PointStamped(
                header=Header(
                    frame_id="map",
                    stamp=Time().to_msg(),
                ),
                point=last_known
            )

            return "succeeded"
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"The following error occured: {e}")
            return "failed"

class FollowPerson(StateMachine):
    def __init__(self):
        # Outcomes align perfectly with your main locate_and_follow_host.py plan
        super().__init__(outcomes=["succeeded", "failed"])

        # Start of following
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
            GetPersonPoint(),
            transitions={
                "succeeded": "SAY_FOLLOW", 
                "failed": "WAIT_FOR_HOST"
            },
        )

        self.add_state(
            "SAY_FOLLOW",
            Say(text="I will now follow you. Lead the way slowly. "),
            transitions={
                "succeeded": "TRACK_AND_NAVIGATE",
                "aborted": "TRACK_AND_NAVIGATE",
                "canceled": "TRACK_AND_NAVIGATE",
            },
        )

        # Tracking and Navigate
        self.add_state(
            "TRACK_AND_NAVIGATE",
            Concurrence(
                states={
                    "tracker": TrackPerson(),
                    "navigator": ContinuousGoToLocation(),
                },
                default_outcome="failed",
                outcome_map={
                    "person_stationary": {
                        "tracker": "person_stationary",
                        "navigator": "canceled",
                    },
                    "person_lost": {
                        "tracker": "person_lost",
                        "navigator": "canceled",
                    },
                },
            ),
            transitions={
                "person_stationary": "ASK_IF_ARRIVED",
                "person_lost": "CALL_LOST_PERSON_BACK",
                "failed": "failed",
            },
        )

        self.add_state(
            "CALL_LOST_PERSON_BACK",
            Say(text="I seam to have lost track of you. I will wait until you are back infront of me. "),
            transitions={
                "succeeded": "UPDATE_POLYGON",
                "aborted": "UPDATE_POLYGON",
                "canceled": "UPDATE_POLYGON",
            },
        )

        # Stationay Person
        self.add_state(
            "ASK_IF_ARRIVED",
            AskAndListen(
                tts_phrase="Say YES if we have arrived. NO if we have not.",
            ),
            transitions={
                "succeeded": "PROCESS_RESPONSE",
                "failed": "ASK_IF_ARRIVED",
            }
        )
        self.add_state(
            "PROCESS_RESPONSE",
            yasmin.CbState(outcomes=["yes", "unknown", "no"], callback=self.parse_arrival_confirmation),
            transitions={
                "yes": "succeeded", 
                "unknown": "FEEDBACK_RESPONSE",
                "no": "SAY_FOLLOW",
            },
        )
        self.add_state(
            "FEEDBACK_RESPONSE",
            Say(text="I didn't quite understand that. "),
            transitions={
                "succeeded": "ASK_IF_ARRIVED",
                "aborted": "ASK_IF_ARRIVED",
                "canceled": "ASK_IF_ARRIVED",
            },
        )

    def parse_arrival_confirmation(self, blackboard):
        response = str(blackboard["transcribed_speech"]).lower()
        yasmin.YASMIN_LOG_INFO(f"Recieved response: {response}")

        if "yes" in response:
            return "yes" 
        elif "no" in response:
            return "no" 
        else:
            return "unknown"

def main():
    rclpy.init()

    yasmin_ros.set_ros_loggers()

    sm = FollowPerson()
    sm.set_sigint_handler(True)
    bb = Blackboard()
    bb["z_sweep_min"] = -10
    bb["z_sweep_max"] = 50

    YasminViewerPub(sm, "Follow_Person")

    outcome = sm(bb)

    yasmin.YASMIN_LOG_INFO(outcome)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
