"""Generic wait state for waiting a desired number of seconds"""

import rclpy
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
        self._logger = rclpy.logging.get_logger("WaitState")

    def execute(self, blackboard):
        try:
            self._logger.info(f"Waiting for {self._wait_time} seconds.")
            sleep(self._wait_time)
            return "succeeded"
        except:
            self._logger.error("Waiting failed")
            return "failed"
