import math
import rclpy
import yasmin
import tf2_ros
from yasmin import State
from yasmin_ros.yasmin_node import YasminNode
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, Point, Quaternion


class ApproachPerson(State):
    def __init__(self, stop_distance=1.0):
        super().__init__(outcomes=["succeeded", "failed"])
        self.stop = stop_distance
        self._node = YasminNode.get_instance()
        self._buf = tf2_ros.Buffer()
        self._tf = tf2_ros.TransformListener(self._buf, self._node)

    def execute(self, blackboard):
        p = blackboard["wave_position"].point  # у map
        try:
            t = self._buf.lookup_transform(
                "map", "base_footprint", rclpy.time.Time(), Duration(seconds=3.0)
            )
        except Exception as e:
            yasmin.YASMIN_LOG_WARN(f"TF map->base failed: {e}")
            return "failed"

        rx, ry = t.transform.translation.x, t.transform.translation.y
        dx, dy = p.x - rx, p.y - ry
        dist = math.hypot(dx, dy)
        yasmin.YASMIN_LOG_INFO(
            f"robot(map)=({rx:.2f},{ry:.2f}) person(map)=({p.x:.2f},{p.y:.2f}) dist={dist:.2f}"
        )

        if dist < 0.5:  # надто близько → хибна детекція (guard ТУТ)
            yasmin.YASMIN_LOG_WARN("rejected: person too close (<0.5m)")
            return "failed"

        ratio = max(0.0, (dist - self.stop) / dist)
        ax, ay = rx + dx * ratio, ry + dy * ratio
        yaw = math.atan2(dy, dx)

        blackboard["location"] = Pose(
            position=Point(x=ax, y=ay, z=0.0),
            orientation=Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
        )
        yasmin.YASMIN_LOG_INFO(f"waypoint(map)=({ax:.2f},{ay:.2f})")
        return "succeeded"
