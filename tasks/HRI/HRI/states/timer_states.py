import rclpy
import yasmin
from smach_ros import RosState
import time


class StartTimer(yasmin.State):
    """State to begin timing an event."""

    def __init__(self, node) -> None:
        super().__init__(
            outcomes=["succeeded", "failed"],
        )
        self.add_output_keys('start_time')

    def execute(self, blackboard):
        try:
            start_time = time.time()
            self._node.get_logger().info("Timer started at: {}".format(start_time))
            blackboard['start_time'] = start_time
            return "succeeded"
        except Exception as e:
            self._node.get_logger().error(f"Error starting timer: {e}")
            return "failed"


class StopTimer(yasmin.State):
    """State to stop timing an event and calculate the duration."""

    def __init__(self, node) -> None:
        super().__init__(
            outcomes=["succeeded", "failed"],
        )
        self.add_input_key('start_time')
        self.add_output_key('duration')
        self.add_output_key('time_text')

    def execute(self, blackboard):
        try:
            end_time = time.time()
            duration = end_time - blackboard['start_time']
            self._node.get_logger().info("Timer stopped. Duration: {}".format(duration))
            mins, secs = divmod(duration, 60)
            blackboard['time_text'] = f"Receptionist took {int(mins)} minutes and {int(secs)} seconds to complete the task."
            blackboard['duration'] = duration
            return "succeeded"
        except Exception as e:
            self._node.get_logger().error(f"Error stopping timer: {e}")
            return "failed"
