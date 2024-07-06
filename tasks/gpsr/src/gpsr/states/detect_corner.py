import smach
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math


class DetectTurn(smach.State):
    """
    State for detecting Tiago is turning around <-- for guiding people.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=["turn_detected", "no_turn"])
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.sub_imu = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.prev_orientation = None
        self.turn_detected = False
        self.turn_threshold = math.radians(45)  # 45 degrees threshold

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        (roll, pitch, yaw) = self.quaternion_to_euler(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        )

        if self.prev_orientation is not None:
            yaw_diff = self.angle_diff(yaw, self.prev_orientation)
            if abs(yaw_diff) > self.turn_threshold:
                rospy.loginfo(
                    "Turn detected: {:.2f} degrees".format(math.degrees(yaw_diff))
                )
                self.turn_detected = True

        self.prev_orientation = yaw

    def imu_callback(self, msg):
        angular_velocity_z = msg.angular_velocity.z
        if abs(angular_velocity_z) > self.turn_threshold:
            rospy.loginfo(
                "Turn detected from IMU: {:.2f} degrees/sec".format(
                    math.degrees(angular_velocity_z)
                )
            )
            self.turn_detected = True

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def angle_diff(self, a, b):
        diff = a - b
        while diff < -math.pi:
            diff += 2 * math.pi
        while diff > math.pi:
            diff -= 2 * math.pi
        return diff

    def execute(self, userdata):
        if self.turn_detected:
            self.turn_detected = False  # Reset flag for next detection
            return "turn_detected"
        else:
            return "no_turn"


class TrackDistance(smach.State):
    """
    State for detecting Tiago moved a certain distance <-- for guiding people.
    """

    def __init__(self, target_distance):
        smach.State.__init__(self, outcomes=["distance_reached", "failed"])
        self.target_distance = target_distance
        self.start_position = None
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def odom_callback(self, msg):
        if self.start_position is None:
            self.start_position = msg.pose.pose.position
        else:
            current_position = msg.pose.pose.position
            distance_travelled = self.calculate_distance(
                self.start_position, current_position
            )
            if distance_travelled >= self.target_distance:
                self.start_position = None  # Reset for next distance tracking
                self.sub_odom.unregister()  # Stop the subscriber
                self.distance_reached = True

    def calculate_distance(self, start, end):
        return math.sqrt((start.x - end.x) ** 2 + (start.y - end.y) ** 2)

    def execute(self, userdata):
        self.distance_reached = False
        while not self.distance_reached:
            rospy.sleep(0.1)
        return "distance_reached"
