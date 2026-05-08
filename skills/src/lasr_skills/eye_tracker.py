import rclpy
from smach_ros import SimpleActionState
from rclpy.action import ActionClient

from lasr_vision_interfaces.action import EyeTracker as EyeTrackerAction


class StartEyeTracker(SimpleActionState):
    def __init__(self, node):
        super().__init__(node=node, action_name='/lasr_vision_eye_tacker/track_eyes', action_spec=EyeTrackerAction, goal_cb=self.create_goal)

    def create_goal(self, userdata, goal_msg):
        goal_msg.person_point = userdata.person_point

        return goal_msg


class StopEyeTracker(SimpleActionState):
    def __init__(self, node):
        super().__init__(node=node, action_name='/lasr_vision_eye_tacker/track_eyes', action_spec=EyeTrackerAction, goal_cb=self.create_goal)
        super()._cancel_goal()