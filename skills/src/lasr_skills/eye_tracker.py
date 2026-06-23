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

        self.add_input_key("person_point")

    def create_goal(self, blackboard):
        goal_msg = EyeTrackerAction.Goal()
        goal_msg.person_point = blackboard["person_point"]

        return goal_msg


class StopEyeTracker(yasmin_ros.ActionState):
    def __init__(self):
        super().__init__(
            action_name="/lasr_vision_eye_tracker/track_eyes",
            action_type=EyeTrackerAction,
            create_goal_handler=self._create_goal,
            response_timeout=1.0,
            maximum_retry=0,
        )

    def _create_goal(self, blackboard):
        return EyeTrackerAction.Goal(cancel=True)
