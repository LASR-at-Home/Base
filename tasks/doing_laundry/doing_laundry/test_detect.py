"""
test_detect.py — standalone perception check (no MoveIt, no SM).
Spins a node, runs BasketPerception.detect_once every second, prints + publishes.

  ros2 run doing_laundry test_detect
  # default topics are the sim's /head_front_camera/... ; for the real robot:
  # ros2 run doing_laundry test_detect --ros-args \
  #   -p basket.depth_topic:=/xtion/depth/image_raw \
  #   -p basket.rgb_topic:=/xtion/rgb/image_raw \
  #   -p basket.info_topic:=/xtion/rgb/camera_info
"""
import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from doing_laundry.states.detect_basket import BasketPerception


def main():
    rclpy.init(args=sys.argv)
    node = Node('basket_detect_test')
    node.declare_parameter('basket.depth_topic', '/head_front_camera/depth/image_raw')
    node.declare_parameter('basket.rgb_topic', '/head_front_camera/rgb/image_raw')
    node.declare_parameter('basket.info_topic', '/head_front_camera/rgb/camera_info')
    per = BasketPerception(
        node,
        depth_topic=node.get_parameter('basket.depth_topic').value,
        rgb_topic=node.get_parameter('basket.rgb_topic').value,
        info_topic=node.get_parameter('basket.info_topic').value)
    exe = MultiThreadedExecutor()
    exe.add_node(node)
    threading.Thread(target=exe.spin, daemon=True).start()
    try:
        while rclpy.ok():
            r = per.detect_once()
            per.publish(r)
            g = tuple(round(v, 3) for v in r.grasp) if r.grasp else None
            node.get_logger().info(
                f'{r.status} | {r.reason} | basket=({r.basket_xy[0]:.2f},{r.basket_xy[1]:.2f}) '
                f'yaw={r.basket_yaw:.2f} rim_z={r.rim_z:.3f} grasp={g}')
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
