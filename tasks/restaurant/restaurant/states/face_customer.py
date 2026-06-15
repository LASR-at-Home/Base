import math
import rclpy
import yasmin
import tf2_ros
from yasmin import State
from yasmin_ros.yasmin_node import YasminNode
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from time import sleep


class FaceCustomer(State):
    def __init__(self, tol=0.1, speed=0.5, cmd_topic="/cmd_vel"):
        super().__init__(outcomes=["succeeded", "failed"])
        self.tol = tol
        self.speed = speed
        self._node = YasminNode.get_instance()
        self._buf = tf2_ros.Buffer()
        self._tf = tf2_ros.TransformListener(self._buf, self._node)
        self._cmd = self._node.create_publisher(Twist, cmd_topic, 10)

    def execute(self, blackboard):
        p = blackboard["wave_position"].point
        for _ in range(200):
            try:
                t = self._buf.lookup_transform(
                    "map", "base_footprint", rclpy.time.Time(), Duration(seconds=1.0)
                )
            except Exception as e:
                yasmin.YASMIN_LOG_WARN(f"TF map->base failed: {e}")
                sleep(0.1)
                continue

            rx, ry = t.transform.translation.x, t.transform.translation.y
            q = t.transform.rotation
            robot_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            target_yaw = math.atan2(p.y - ry, p.x - rx)
            err = math.atan2(
                math.sin(target_yaw - robot_yaw), math.cos(target_yaw - robot_yaw)
            ) 

            if abs(err) < self.tol:
                self._cmd.publish(Twist()) 
                yasmin.YASMIN_LOG_INFO(f"facing customer (err={err:.2f})")
                return "succeeded"

            tw = Twist()
            tw.angular.z = max(-self.speed, min(self.speed, 1.5 * err))
            self._cmd.publish(tw)
            sleep(0.1)

        self._cmd.publish(Twist())
        yasmin.YASMIN_LOG_WARN("face customer: timeout")
        return "succeeded"