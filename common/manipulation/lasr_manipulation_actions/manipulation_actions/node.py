#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from rclpy.time import Time as ROS2Time
import tf2_ros
from tf2_geometry_msgs import do_transform_point

from lasr_vision_interfaces.msg import Detection3DArray

from lasr_manipulation_actions.grasp_planner import GraspPlanner


class ManipulationNode(Node):
    def __init__(self):
        super().__init__('lasr_manipulation_actions')
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('pregrasp_motion', 'pregrasp')
        self.declare_parameter('align_y_tolerance', 0.05)
        self.declare_parameter('align_y_step_coarse', 0.15)
        self.declare_parameter('align_y_step_fine', 0.10)
        self.declare_parameter('align_max_iterations', 100)
        self.declare_parameter('grasp_z', 0.77)
        self.declare_parameter('retreat_z', 0.95)

        self.target_frame = self.get_parameter('target_frame').value
        params = {
            'pregrasp_motion':      self.get_parameter('pregrasp_motion').value,
            'align_y_tolerance':    self.get_parameter('align_y_tolerance').value,
            'align_y_step_coarse':  self.get_parameter('align_y_step_coarse').value,
            'align_y_step_fine':    self.get_parameter('align_y_step_fine').value,
            'align_max_iterations': self.get_parameter('align_max_iterations').value,
            'grasp_z':              self.get_parameter('grasp_z').value,
            'retreat_z':            self.get_parameter('retreat_z').value,
        }

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.planner = GraspPlanner(self, params)

        self._latest_centroids = Detection3DArray()
        self._busy = False

        self.create_subscription(Detection3DArray, '/object_centroids', self._on_centroids, 10)
        self.create_subscription(String, '/command', self._on_command, 10)

        self.get_logger().info('ManipulationNode ready. Commands: home | init_grasp | grasp <object>')

    def _on_centroids(self, msg: Detection3DArray):
        self._latest_centroids = msg

    def _on_command(self, msg: String):
        if self._busy:
            self.get_logger().warn('Busy, ignoring command')
            return

        cmd = msg.data.strip().lower()
        self._busy = True
        try:
            if cmd == 'home':
                self.planner.go_home()

            elif cmd == 'init_grasp':
                self.planner.init_grasp()

            elif cmd.startswith('grasp '):
                object_name = cmd[len('grasp '):].strip()
                self._do_grasp(object_name)

            else:
                self.get_logger().warn(f'Unknown command: "{cmd}". Use: home | init_grasp | grasp <object>')
        except Exception as e:
            self.get_logger().error(f'Command "{cmd}" failed: {e}')
        finally:
            self._busy = False

    def _do_grasp(self, object_name: str):
        match = None
        for det in self._latest_centroids.detections:
            if object_name.lower() in det.name.lower():
                match = det
                break

        if match is None:
            self.get_logger().warn(f'Object "{object_name}" not found in /object_centroids')
            return

        self.get_logger().info(f'Grasping "{match.name}" at ({match.point.x:.2f}, {match.point.y:.2f}, {match.point.z:.2f})')

        point_stamped = PointStamped()
        point_stamped.header.frame_id = 'map'
        point_stamped.header.stamp = ROS2Time(seconds=0).to_msg()
        point_stamped.point = match.point

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, 'map',
                ROS2Time(seconds=0),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            point_base = do_transform_point(point_stamped, transform).point
        except Exception as e:
            self.get_logger().error(f'TF transform failed: {e}')
            return

        self.planner.pick(point_base)


def main(args=None):
    rclpy.init(args=args)
    node = ManipulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
