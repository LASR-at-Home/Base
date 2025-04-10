"""Generic wait state for waiting a desired number of seconds"""

import rclpy
from ros_state import RosState
from time import sleep


class Wait(RosState):
    def __init__(self, node, wait_time: int):
        """
        Args:
            wait_time (int): Number of seconds to wait for and remain idle
        """
        super().__init__(self, node, outcomes=["succeeded", "failed"])

        if not rclpy.ok():
            rclpy.init()

        self._wait_time = wait_time
        self._logger = rclpy.logging.get_logger("WaitState")

    def execute(self, userdata):
        try:
            self._logger.info(f"Waiting for {self._wait_time} seconds.")
            sleep(self._wait_time)
            return "succeeded"
        except:
            self._logger.error("Waiting failed")
            return "failed"
