import rclpy
from smach_ros import RosState
import time


class StartTimer(RosState):
    """State to begin timing an event."""

    def __init__(self, node) -> None:
        super().__init__(
            node=node, outcomes=["succeeded", "failed"], output_keys=["start_time"]
        )

    def execute(self, userdata):
        try:
            start_time = time.time()
            self.node.get_logger().info("Timer started at: {}".format(start_time))
            userdata.start_time = start_time
            return "succeeded"
        except Exception as e:
            self.node.get_logger().error(f"Error starting timer: {e}")
            return "failed"


class StopTimer(RosState):
    """State to stop timing an event and calculate the duration."""

    def __init__(self, node) -> None:
        super().__init__(
            node=node,
            outcomes=["succeeded", "failed"],
            input_keys=["start_time"],
            output_keys=["duration", "time_text"],
        )

    def execute(self, userdata):
        try:
            end_time = time.time()
            duration = end_time - userdata.start_time
            self.node.get_logger().info("Timer stopped. Duration: {}".format(duration))
            mins, secs = divmod(duration, 60)
            userdata.time_text = f"Receptionist took {int(mins)} minutes and {int(secs)} seconds to complete the task."
            userdata.duration = duration
            return "succeeded"
        except Exception as e:
            self.node.get_logger().error(f"Error stopping timer: {e}")
            return "failed"