import rclpy
import yasmin

import time


class StartTimer(yasmin.State):
    """State to begin timing an event."""

    def __init__(self) -> None:
        super().__init__(
            outcomes=["succeeded", "failed"],
        )
        self.add_output_keys('start_time')

    def execute(self, blackboard):
        try:
            start_time = time.time()
            yasmin.YASMIN_LOG_INFO("Timer started at: {}".format(start_time))
            blackboard['start_time'] = start_time
            return "succeeded"
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Error starting timer: {e}")
            return "failed"


class StopTimer(yasmin.State):
    """State to stop timing an event and calculate the duration."""

    def __init__(self) -> None:
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
            yasmin.YASMIN_LOG_INFO("Timer stopped. Duration: {}".format(duration))
            mins, secs = divmod(duration, 60)
            blackboard['time_text'] = f"Receptionist took {int(mins)} minutes and {int(secs)} seconds to complete the task."
            blackboard['duration'] = duration
            return "succeeded"
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Error stopping timer: {e}")
            return "failed"
