"""
Scripted test person for Gazebo (sim testing only).

Spawns a person model with the gazebo_ros_planar_move plugin and walks it through a
scenario: a list of steps, each either a waypoint or a pause.

    steps:                                   # Gazebo world frame (= robot odom at start)
      - {x: 1.0, y: 0.0, speed: 0.5}
      - {pause: 3.0}
      - {x: 2.0, y: 1.5, speed: 0.8}
      - {robot_front: 1.5, speed: 0.6}     # walk to 1.5 m in front of the robot

The model has a solid collision cylinder, so it shows up in the laser scan and Nav2
costmaps like a real person. Ground truth is published on /<name>/ground_truth.

The walk starts after `start_delay` seconds, or when std_msgs/Empty arrives on
/<name>/go when start_delay < 0.
"""

import math
import time

import rclpy
import yaml
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String

MESHES = {
    # Same mesh and offsets as the person in the house world.
    "standing": (  # mesh faces the model -y axis -> yaw_offset pi/2
        "https://fuel.ignitionrobotics.org/1.0/openrobotics/models/"
        "standing person/3/files/meshes/standing.dae",
        "0 0 0.02 0.04 0 0",
        1.0,
    ),
    # Gazebo's actor skin (a man in a suit), so a distractor looks different.
    "actor": ("file:///usr/share/gazebo-11/media/models/stand.dae", "0 0 0 0 0 0", 1.0),
}

SDF_TEMPLATE = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <link name="link">
      <inertial>
        <pose>0 0 0.9 0 0 0</pose>
        <mass>70</mass>
        <inertia><ixx>20</ixx><iyy>20</iyy><izz>2</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="body">
        <pose>0 0 0.85 0 0 0</pose>
        <geometry><cylinder><radius>0.22</radius><length>1.7</length></cylinder></geometry>
        <surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface>
      </collision>
      <visual name="visual">
        <pose>{mesh_pose}</pose>
        <geometry><mesh><uri>{mesh_uri}</uri><scale>{s} {s} {s}</scale></mesh></geometry>
      </visual>
    </link>
    <plugin name="planar_move" filename="libgazebo_ros_planar_move.so">
      <ros><namespace>/{name}</namespace></ros>
      <update_rate>50</update_rate>
      <publish_rate>20</publish_rate>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>false</publish_odom_tf>
      <odometry_frame>world</odometry_frame>
      <robot_base_frame>{name}</robot_base_frame>
    </plugin>
  </model>
</sdf>
"""


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class ScriptedPerson(Node):
    def __init__(self):
        super().__init__("scripted_person")
        self.declare_parameter("name", "test_person")
        self.declare_parameter("mesh", "standing")
        self.declare_parameter("x", 1.5)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", math.pi)
        self.declare_parameter("yaw_offset", math.pi / 2)  # mesh facing correction
        self.declare_parameter("scenario", "")
        self.declare_parameter("start_delay", -1.0)
        self.declare_parameter("respawn", True)

        self.name = self.get_parameter("name").value
        self.yaw_offset = self.get_parameter("yaw_offset").value
        scenario = self.get_parameter("scenario").value
        self.steps = []
        self.start_pose = (
            self.get_parameter("x").value,
            self.get_parameter("y").value,
            self.get_parameter("yaw").value,
        )
        if scenario:
            with open(scenario) as f:
                section = yaml.safe_load(f)[self.name]
            self.steps = section["steps"]
            if "start" in section:
                st = section["start"]
                self.start_pose = (st["x"], st["y"], st.get("yaw", 0.0))

        self.cmd_pub = self.create_publisher(Twist, f"/{self.name}/cmd_vel", 10)
        self.gt_pub = self.create_publisher(PoseStamped, f"/{self.name}/ground_truth", 10)
        self.status_pub = self.create_publisher(String, f"/{self.name}/status", 10)
        self.create_subscription(Odometry, f"/{self.name}/odom", self._odom_cb, 10)
        self.robot = None
        self.create_subscription(Odometry, "/ground_truth_odom", self._robot_cb, 10)
        self.create_subscription(Empty, f"/{self.name}/go", self._go_cb, 10)

        self.pose = None
        self.started = False
        self.step_idx = 0
        self.pause_until = None
        self.status = "waiting"

        self._spawn()
        delay = self.get_parameter("start_delay").value
        if delay >= 0:
            self.create_timer(delay, self._start_once)
        self.create_timer(0.05, self._tick)

    # ---- setup ----

    def _spawn(self):
        mesh_uri, mesh_pose, scale = MESHES[self.get_parameter("mesh").value]
        sdf = SDF_TEMPLATE.format(
            name=self.name, mesh_uri=mesh_uri, mesh_pose=mesh_pose, s=scale
        )
        if self.get_parameter("respawn").value:
            delete = self.create_client(DeleteEntity, "/delete_entity")
            if delete.wait_for_service(timeout_sec=5.0):
                fut = delete.call_async(DeleteEntity.Request(name=self.name))
                rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

        spawn = self.create_client(SpawnEntity, "/spawn_entity")
        if not spawn.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("/spawn_entity not available")
        x, y, yaw = self.start_pose
        req = SpawnEntity.Request(name=self.name, xml=sdf)
        req.initial_pose = Pose()
        req.initial_pose.position.x = float(x)
        req.initial_pose.position.y = float(y)
        req.initial_pose.orientation.z = math.sin((yaw + self.yaw_offset) / 2)
        req.initial_pose.orientation.w = math.cos((yaw + self.yaw_offset) / 2)
        fut = spawn.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        self.get_logger().info(f"Spawned {self.name}: {fut.result()}")

    def _robot_cb(self, msg: Odometry):
        self.robot = (msg.pose.pose.position.x, msg.pose.pose.position.y, _yaw(msg.pose.pose.orientation))

    def _start_once(self):
        if not self.started:
            self._go_cb(None)

    def _go_cb(self, _):
        self.get_logger().info(f"{self.name}: starting scenario ({len(self.steps)} steps)")
        self.started = True

    def _odom_cb(self, msg: Odometry):
        self.pose = msg.pose.pose
        gt = PoseStamped()
        gt.header = msg.header
        gt.header.frame_id = "world"
        gt.pose = msg.pose.pose
        self.gt_pub.publish(gt)

    # ---- control ----

    def _tick(self):
        self.status_pub.publish(String(data=self.status))
        cmd = Twist()
        if not self.started or self.pose is None or self.step_idx >= len(self.steps):
            if self.started and self.step_idx >= len(self.steps):
                self.status = "done"
            self.cmd_pub.publish(cmd)
            return

        step = self.steps[self.step_idx]
        now = self.get_clock().now().nanoseconds * 1e-9  # sim time with use_sim_time
        if "pause" in step:
            self.status = "paused"
            if self.pause_until is None:
                self.pause_until = now + float(step["pause"])
            if now >= self.pause_until:
                self.pause_until = None
                self.step_idx += 1
            self.cmd_pub.publish(cmd)
            return

        if "robot_front" in step and "x" not in step:
            if self.robot is None:
                self.cmd_pub.publish(cmd)
                return
            rx, ry, ryaw = self.robot  # fixed once, when this step starts
            step["x"] = rx + step["robot_front"] * math.cos(ryaw)
            step["y"] = ry + step["robot_front"] * math.sin(ryaw)
            self.get_logger().info(f"walking to the robot's front: ({step['x']:.2f}, {step['y']:.2f})")

        self.status = "walking"
        px, py = self.pose.position.x, self.pose.position.y
        yaw = _yaw(self.pose.orientation) - self.yaw_offset
        dx, dy = step["x"] - px, step["y"] - py
        dist = math.hypot(dx, dy)
        if dist < 0.08:
            self.step_idx += 1
            self.cmd_pub.publish(cmd)
            return

        speed = min(float(step.get("speed", 0.6)), 1.5 * dist + 0.05)
        heading = math.atan2(dy, dx)
        # planar_move takes body-frame velocities: walk towards the waypoint while
        # turning the body to face the direction of travel.
        rel = heading - (yaw + self.yaw_offset)
        cmd.linear.x = speed * math.cos(rel)
        cmd.linear.y = speed * math.sin(rel)
        cmd.angular.z = max(-2.0, min(2.0, 3.0 * _wrap(heading - yaw)))
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = ScriptedPerson()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cmd_pub.publish(Twist())
    node.destroy_node()
    rclpy.try_shutdown()
