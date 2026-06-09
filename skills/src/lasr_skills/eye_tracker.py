import rclpy
import yasmin_ros
from rclpy.action import ActionClient

from lasr_vision_interfaces.action import EyeTracker as EyeTrackerAction


class StartEyeTracker(yasmin_ros.ActionState):
    def __init__(self):
        super().__init__(
            action_name="/lasr_vision_eye_tracker/track_eyes",
            action_type=EyeTrackerAction,
            create_goal_handler=self.create_goal,
            response_timeout=1.0,
            maximum_retry=0,
        )

    def create_goal(self, blackboard):
        goal_msg = EyeTrackerAction.Goal()
        goal_msg.person_point = blackboard["person_point"]

        return goal_msg


class StopEyeTracker(yasmin_ros.ActionState):
    def __init__(self):
        super().__init__(
            action_name="/lasr_vision_eye_tracker/track_eyes",
            action_spec=EyeTrackerAction,
            goal_cb=self.create_goal,
            create_goal_handler=self.cancel_goal,
        )

        super().cancel_state()
