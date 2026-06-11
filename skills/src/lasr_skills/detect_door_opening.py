import time
from typing import Optional
import numpy as np
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription

import yasmin
from yasmin import StateMachine, State, Blackboard
import yasmin_ros

from sensor_msgs.msg import LaserScan


class DetectDoorOpening(State):
    """State to detect when a door has been opened using LIDAR data."""

    _scan_topic: str
    _scan_subscriber: Optional[Subscription]
    _door_opened: bool
    _opened_delta: float
    _timeout: float
    _initial_mean_distance: Optional[float]

    def __init__(
        self,
        lasr_scan_topic: str = "/scan_raw",
        opened_delta: float = 0.5,
        timeout: float = 15.0,
    ):
        super().__init__(outcomes=["door_opened", "failed"])

        self._node = yasmin_ros.logger_node
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
            yasmin.YASMIN_LOG_WARN(
                "Initial mean distance not set. Cannot determine if door is opened."
            )
            return

        if np.isnan(mean_distance):
            yasmin.YASMIN_LOG_WARN("Laser scan mean distance is NaN.")
            return

        if mean_distance - self._initial_mean_distance > self._opened_delta:
            yasmin.YASMIN_LOG_INFO("Door has been opened.")
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

    def execute(self, blackboard):
        yasmin.YASMIN_LOG_INFO("Waiting for door to open...")
        self._door_opened = False

        initial_scan = self._wait_for_initial_scan(timeout=5.0)
        if initial_scan is None:
            yasmin.YASMIN_LOG_WARN(
                "No laser scan received while waiting for initial door state."
            )
            return "failed"

        self._initial_mean_distance = self._compute_mean_distance(initial_scan)
        if np.isnan(self._initial_mean_distance):
            yasmin.YASMIN_LOG_WARN(
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

        yasmin.YASMIN_LOG_WARN("Door did not open before timeout.")
        return "failed"


def main():
    rclpy.init()
    node = rclpy.create_node("hri")
    yasmin_ros.set_ros_loggers(node)

    try:
        sm = StateMachine(outcomes=["succeeded", "failed"])
        sm.add_state(
            "DETECT_DOOR_OPENING",
            DetectDoorOpening(),
            transitions={"door_opened": "succeeded", "failed": "failed"},
        )
        bb = Blackboard()
        outcome = sm(bb)
        yasmin.YASMIN_LOG_INFO(outcome)
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
