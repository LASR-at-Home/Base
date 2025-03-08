"""Generic wait state for waiting a desired number of seconds"""

import rclpy
import smach
from time import sleep
from lasr_skills import AccessNode


class Wait(smach.State):
    def __init__(self, wait_time: int):
        """
        Args:
            wait_time (int): Number of seconds to wait for and remain idle
        """
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

        self.node = AccessNode.get_node()

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

# class WaitStateNode(Node):
#     def __init__(self):
#         super().__init__("wait_state_node")

#         sm = smach.StateMachine(outcomes=["succeeded", "failed"])

#         with sm:
#             smach.StateMachine.add("WAIT", Wait(wait_time=5), transitions={"succeeded": "succeeded", "failed": "failed"})

#         outcome = sm.execute()

# def main():
#     rclpy.init()
#     node = WaitStateNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()


