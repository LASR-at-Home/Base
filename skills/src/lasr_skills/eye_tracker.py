import smach
import actionlib

from lasr_vision_msgs.msg import (
    EyeTrackerAction,
    EyeTrackerGoal,
)


class StartEyeTracker(smach.State):

    _action_client: actionlib.SimpleActionClient

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["person_point"],
        )
        self._action_client = actionlib.SimpleActionClient(
            "/lasr_vision_eye_tracker/track_eyes", EyeTrackerAction
        )
        self._action_client.wait_for_server()

    def execute(self, userdata):
        goal = EyeTrackerGoal()
        goal.person_point = userdata.person_point
        self._action_client.send_goal(goal)
        return "succeeded"


class StopEyeTracker(smach.State):

    _action_client: actionlib.SimpleActionClient

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["person_point"],
            output_keys=["eyes"],
        )
        self._action_client = actionlib.SimpleActionClient(
            "/lasr_vision_eye_tracker/track_eyes", EyeTrackerAction
        )
        self._action_client.wait_for_server()

    def execute(self, userdata):
        # Send a cancel request to the action server
        self._action_client.cancel_all_goals()
        return "succeeded"
