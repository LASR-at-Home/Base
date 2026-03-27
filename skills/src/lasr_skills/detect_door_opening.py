import time
from typing import Optional

import numpy as np
import rclpy
import smach
from rclpy.node import Node
from rclpy.subscription import Subscription
from sensor_msgs.msg import LaserScan


class DetectDoorOpening(smach.State):
    """State to detect when a door has been opened using LIDAR data."""

    _scan_topic: str
    _scan_subscriber: Optional[Subscription]
    _door_opened: bool
    _opened_delta: float
    _timeout: float
    _initial_mean_distance: Optional[float]

    def __init__(
        self,
        node: Node,
        lasr_scan_topic: str = "/scan_raw",
        opened_delta: float = 0.5,
        timeout: float = 15.0,
    ):
        super().__init__(outcomes=["door_opened", "failed"])

        self._node = node
        self._scan_topic = lasr_scan_topic
        self._scan_subscriber = None
        self._door_opened = False
        self._opened_delta = opened_delta
        self._timeout = timeout
        self._initial_mean_distance = None

    @staticmethod
    def _compute_mean_distance(scan: LaserScan) -> float:
        range_data = np.array(scan.ranges, dtype=float)
        invalid_mask = (
            np.isinf(range_data)
            | (range_data == scan.range_max)
            | (range_data == scan.range_min)
        )
        range_data[invalid_mask] = np.nan
        return float(np.nanmean(range_data))

    def _is_door_opened(self, msg: LaserScan) -> None:
        if self._door_opened:
            return

        mean_distance = self._compute_mean_distance(msg)

        if self._initial_mean_distance is None:
            self._node.get_logger().warn(
                "Initial mean distance not set. Cannot determine if door is opened."
            )
            return

        if np.isnan(mean_distance):
            self._node.get_logger().warn("Laser scan mean distance is NaN.")
            return

        self._node.get_logger().info(f"Current mean distance: {mean_distance:.2f}")
        if mean_distance - self._initial_mean_distance > self._opened_delta:
            self._node.get_logger().info("Door has been opened.")
            self._door_opened = True

    def _wait_for_initial_scan(self, timeout: float = 5.0) -> Optional[LaserScan]:
        initial_scan: Optional[LaserScan] = None

        def _capture_first_scan(msg: LaserScan) -> None:
            nonlocal initial_scan
            if initial_scan is None:
                initial_scan = msg

        temp_sub = self._node.create_subscription(
            LaserScan,
            self._scan_topic,
            _capture_first_scan,
            1,
        )

        start_time = time.time()
        while (
            rclpy.ok() and initial_scan is None and (time.time() - start_time) < timeout
        ):
            rclpy.spin_once(self._node, timeout_sec=0.1)

        self._node.destroy_subscription(temp_sub)
        return initial_scan

    def execute(self, userdata):
        self._node.get_logger().info("Waiting for door to open...")
        self._door_opened = False

        initial_scan = self._wait_for_initial_scan(timeout=5.0)
        if initial_scan is None:
            self._node.get_logger().warn(
                "No laser scan received while waiting for initial door state."
            )
            return "failed"

        self._initial_mean_distance = self._compute_mean_distance(initial_scan)
        if np.isnan(self._initial_mean_distance):
            self._node.get_logger().warn(
                "Initial laser scan mean distance is NaN. Failing door detection."
            )
            return "failed"

        self._scan_subscriber = self._node.create_subscription(
            LaserScan,
            self._scan_topic,
            self._is_door_opened,
            1,
        )

        start_time = time.time()
        while (
            rclpy.ok()
            and (not self._door_opened)
            and ((time.time() - start_time) < self._timeout)
        ):
            rclpy.spin_once(self._node, timeout_sec=0.1)

        if self._scan_subscriber is not None:
            self._node.destroy_subscription(self._scan_subscriber)
            self._scan_subscriber = None

        if self._door_opened:
            return "door_opened"

        self._node.get_logger().warn("Door did not open before timeout.")
        return "failed"


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("detect_door_opening")

    try:
        detect = DetectDoorOpening(node=node)
        sm = smach.StateMachine(outcomes=["succeeded", "failed"])
        with sm:
            smach.StateMachine.add(
                "DETECT_DOOR_OPENING",
                detect,
                transitions={"door_opened": "succeeded", "failed": "failed"},
            )
        sm.execute()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
