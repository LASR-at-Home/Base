"""
Minimal stand-in for the FollowPerson state machine (sim testing of the follow pipeline):
lock the host -> start following -> stop when the host is stationary.
"""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool


class FollowDriver(Node):
    def __init__(self):
        super().__init__("follow_driver")
        self.state = None
        self.create_subscription(String, "/person_tracker/host_state", self._cb, 10)
        self.lock = self.create_client(Trigger, "/person_tracker/lock_host")
        self.enable = self.create_client(SetBool, "/person_tracker/enable")
        self.start = self.create_client(Trigger, "/follow_manager/start")
        self.stop = self.create_client(Trigger, "/follow_manager/stop")

    def _cb(self, msg):
        self.state = json.loads(msg.data)

    def call(self, cli):
        cli.wait_for_service()
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut)
        return fut.result()

    def run(self):
        self.enable.wait_for_service()
        fut = self.enable.call_async(SetBool.Request(data=True))
        rclpy.spin_until_future_complete(self, fut)
        while rclpy.ok():
            res = self.call(self.lock)
            if res.success:
                break
            self.get_logger().info(f"lock: {res.message}", throttle_duration_sec=2.0)
            t = time.monotonic()
            while time.monotonic() - t < 0.5:
                rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("EVENT locked host")
        self.call(self.start)
        self.get_logger().info("EVENT following")
        last = None
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            s = self.state
            if s is None:
                continue
            if s["status"] != last:
                self.get_logger().info(f"EVENT host {s['status']}")
                last = s["status"]
            if s["stationary"]:
                self.get_logger().info(f"EVENT host stationary ({s['still_for']:.1f}s, travelled {s['travelled']:.1f}m): ARRIVED")
                self.call(self.stop)
                break
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)


def main():
    rclpy.init()
    node = FollowDriver()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
