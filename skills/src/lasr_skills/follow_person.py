import math

import rclpy
from rclpy.action import ActionClient
from rclpy.time import Time

from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, PointStamped
from std_msgs.msg import Header
from nav2_msgs.action import NavigateToPose

from yasmin import State, Blackboard, StateMachine
import yasmin_ros

from .detect_3d import Detect3D
from .wait import Wait


'''

def euclidean_distance(point1: Point, point2: Point) -> float:
        """Calculates the Euclidean distance between two points."""
        return np.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)

    new_detections: List[Detection3D] = []

    try:
        for detection in blackboard["detections_3d"]:
            if detection in blackboard["detected_objects"]:
                continue

            # Check if the detection is a new object
            is_new_object = True
            for existing_detection in blackboard["detected_objects"]:
                if (
                    existing_detection.name == detection.name
                    and euclidean_distance(
                        existing_detection.point, detection.point
                    )
                    < self._min_new_object_dist
                ):
                    yasmin_ros.logger_node.get_logger().info(
                        f"Detected object {detection.name} is too close to existing object {existing_detection.name}. Not counting as new."
                    )
                    is_new_object = False
                    break
'''
class PublishPersonPose(PublisherState):

    def __init__(self):
        super().__init__(PointStamped, "/person_pose", self.create_publisher)
        self.add_input_key("detections_3d")

        self.robot_pose = self._node.create_subscriber(PoseWithCovarianceStamped, '/amcl_pose', self.robot_cb, 10)

    def create_publisher(self, blackboard):
        detections=blackboard["detections_3d"]

        person_point = Point()
        if len(detections) == 0:
            pass # Exit
        elif len(detections) > 1:
            pass # Get person pose who is closested to the robot
        else:
            pass # something

        msg = PointStamped(
            header=Header(
                frame_id="map",
                stamp=Time().to_msg(),
            ),
            point=person_point,
        )
        return msg

class GetPersonLocation(StateMachine):
    def __init__(self):
        pass
        self.add_state(
            "DETECT_3D",
            Detect3D(filter=["person"]),
            transitions={
                "succeeded": "FILTER_DETECTIONS", 
                "failed": "failed"
                }
            )
        
        self.add_state(
            "PUBLISH_PERSON",
            PublishPersonPose(),
            transitions={
                "succeeded": "WAIT",
                "aborted": "DETECT_3D",
                "canceled": "DETECT_3D",
            },
            )
        self.add_state(
            "WAIT",
            Wait(2),
            transitions={
                "succeeded": "DETECT_3D", 
                "failed": "DETECT_3D"
                }
            )


class DynamicFollowPerson(State):
    def __init__(self, target_distance=1.5):
        # "succeeded": Never really succeeds unless we decide to stop following
        # "person_lost": Detector stopped seeing the person
        # "failed": Nav2 crashed
        super().__init__(outcomes=["succeeded", "person_lost", "failed"])
        self.node = yasmin_ros.logger_node
        self.target_distance = target_distance
        
        # Action Client for NavigateToPose
        self.nav_client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self.active_goal_handle = None
        
        # State variables
        self.last_seen_time = self.node.get_clock().now()
        self.latest_person_pose = None
        self.latest_robot_pose = None

        # --- TODO: Add your Subscriptions here ---
        self.person_sub = self.node.create_subscription(PointStamped, '/person_pose', self.person_cb, 10)

        #QOS: RELIABLE, KEEP_LAST, TRANSIENT_LOCAL
        self.robot_sub = self.node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.robot_cb, 10)

    def person_cb(self, msg):
        self.latest_person_pose = msg
        self.last_seen_time = self.node.get_clock().now()

    def robot_cb(self, msg):
        self.latest_robot_pose = msg

    def calculate_offset_goal(self):
        """ Calculates a goal exactly 1.5m away from the person """
        px = self.latest_person_pose.pose.position.x
        py = self.latest_person_pose.pose.position.y

        rx = self.latest_robot_pose.pose.pose.position.x
        ry = self.latest_robot_pose.pose.pose.position.y

        # Calculate distance between robot and person
        dx = px - rx
        dy = py - ry
        distance = math.sqrt(dx**2 + dy**2)

        if distance <= self.target_distance:
            return None, distance # Too close!

        # Calculate the ratio to pull the point back by 1.5 meters
        ratio = (distance - self.target_distance) / distance
        
        goal = PoseStamped()
        goal.header.frame_id = "map" # Assuming your poses are in the map frame
        goal.pose.position.x = rx + (dx * ratio)
        goal.pose.position.y = ry + (dy * ratio)
        
        # Make the robot point at the person
        theta = math.atan2(dy, dx)
        goal.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.orientation.w = math.cos(theta / 2.0)

        return goal, distance

    def execute(self, blackboard: Blackboard):
        self.node.get_logger().info("Starting Dynamic Follow mode...")

        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.node.get_logger().error("NavigateToPose server not available!")
            return "failed"

        rate = self.node.create_rate(2.0) # We update the goal 2 times a second

        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

            # 1. Check if we lost the person
            time_since_last_seen = (self.node.get_clock().now() - self.last_seen_time).nanoseconds / 1e9
            if time_since_last_seen > 3.0:
                self.node.get_logger().warn("Lost person! Stopping.")
                if self.active_goal_handle:
                    self.active_goal_handle.cancel_goal_async()
                return "person_lost"

            # 2. Ensure we have data
            if not self.latest_person_pose or not self.latest_robot_pose:
                continue

            # 3. Calculate target and distance
            target_goal, distance = self.calculate_offset_goal()

            # 4. Apply Deadband Logic
            if distance <= self.target_distance:
                # We are within 1.5m. Stop the robot if it's currently moving.
                if self.active_goal_handle:
                    self.node.get_logger().info("Inside 1.5m deadband. Stopping robot.")
                    self.active_goal_handle.cancel_goal_async()
                    self.active_goal_handle = None
            
            elif distance > self.target_distance + 0.2: 
                # Person is further than 1.7m (0.2m buffer prevents jitter). Send a new goal!
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose = target_goal
                
                # Send the goal asynchronously so we don't block the loop
                send_goal_future = self.nav_client.send_goal_async(goal_msg)
                
                # Add a callback to store the goal handle once Nav2 accepts it
                def goal_response_callback(future):
                    handle = future.result()
                    if handle.accepted:
                        self.active_goal_handle = handle

                send_goal_future.add_done_callback(goal_response_callback)

            rate.sleep()

        return "failed"
    

class FollowHost(StateMachine):
    pass # Runs the DynamicFollowPerson and GetPersonLocation concurrently untill DynamicFollowPerson finishes