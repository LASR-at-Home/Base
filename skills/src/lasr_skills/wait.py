"""Generic wait state for waiting a desired number of seconds"""

import time
import smach
import rclpy


class Wait(smach.State):  
    def __init__(self, wait_time: int):
        """
        Args:
            wait_time (int): Number of seconds to wait for and remain idle
        """
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self._wait_time = wait_time
        self._logger = rclpy.logging.get_logger("WaitState")

    def execute(self, userdata) -> str:
        try:
            self._logger.info(f"Waiting for {self._wait_time} seconds.")
            time.sleep(self._wait_time)
            return "succeeded"
        except Exception as e:
            self._logger.error(f"Waiting failed: {e}")
            return "failed" 