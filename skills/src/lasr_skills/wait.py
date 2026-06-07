"""Generic wait state for waiting a desired number of seconds"""

import rclpy
import yasmin
from yasmin import State
from time import sleep


class Wait(State):
    def __init__(self, wait_time: int):
        """
        Args:
            wait_time (int): Number of seconds to wait for and remain idle
        """
        super().__init__(outcomes=["succeeded", "failed"])

        self._wait_time = wait_time

    def execute(self, blackboard):
        try:
            yasmin.YASMIN_LOG_INFO(f"Waiting for {self._wait_time} seconds.")
            sleep(self._wait_time)
            return "succeeded"
        except:
            yasmin.YASMIN_LOG_ERROR("Waiting failed")
            return "failed"
