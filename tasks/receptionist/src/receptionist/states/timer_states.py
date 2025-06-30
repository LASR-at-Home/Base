import rospy
import smach
import time


class StartTimer(smach.State):
    """State to begin timing an event."""

    def __init__(self) -> None:
        smach.State.__init__(
            self, outcomes=["succeeded", "failed"], output_keys=["start_time"]
        )

    def execute(self, userdata):
        try:
            userdata.start_time = time.time()
            rospy.loginfo("Timer started at: {}".format(userdata.start_time))
            return "succeeded"
        except Exception as e:
            rospy.logerr(f"Error starting timer: {e}")
            return "failed"


class StopTimer(smach.State):
    """State to stop timing an event and calculate the duration."""

    def __init__(self) -> None:
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["start_time"],
            output_keys=["duration", "time_text"],
        )

    def execute(self, userdata):
        try:
            end_time = time.time()
            userdata.duration = end_time - userdata.start_time
            rospy.loginfo("Timer stopped. Duration: {}".format(userdata.duration))
            mins, secs = divmod(userdata.duration, 60)
            userdata.time_text = f"Receptionist took {int(mins)} minutes and {int(secs)} seconds to complete the task."
            return "succeeded"
        except Exception as e:
            rospy.logerr(f"Error stopping timer: {e}")
            return "failed"
