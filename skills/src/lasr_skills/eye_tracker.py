import rclpy
import yasmin_ros
import yasmin
from rclpy.action import ActionClient

from lasr_vision_interfaces.action import EyeTracker as EyeTrackerAction
from rclpy.callback_groups import ReentrantCallbackGroup


class StartEyeTracker(yasmin_ros.ActionState):
    def __init__(self):
        super().__init__(
            action_name="/lasr_vision_eye_tracker/track_eyes",
            action_type=EyeTrackerAction,
            create_goal_handler=self.create_goal,
            feedback_handler=self.handle_feedback,
            callback_group=ReentrantCallbackGroup()
        )

        self.add_input_key("person_point")

    def handle_feedback(self, blackboard, feedback):
        if feedback.running and blackboard['cancel_eye_tracker']:
            yasmin.YASMIN_LOG_INFO('Cancelling current eye tracker')
            self.cancel_state()

    def create_goal(self, blackboard):
        goal_msg = EyeTrackerAction.Goal()
        goal_msg.person_point = blackboard["person_point"]
        blackboard['cancel_eye_tracker'] = False

        return goal_msg


class StopEyeTracker(yasmin.CbState):
    def __init__(self):
        super().__init__(
            outcomes=['succeeded', 'failed'],
            callback=self.cancel_goal,
        )

    def cancel_goal(self, blackboard):
        try:
            blackboard['cancel_eye_tracker'] = True
            return 'succeeded'
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Error raised: {e}')
            return 'failed'
