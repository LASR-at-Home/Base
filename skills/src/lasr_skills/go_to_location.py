from typing import Union
import rclpy
from ros_state import RosState
from geometry_msgs.msg import Pose, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Header


class GoToLocation(RosState):
    def __init__(self, node, location: Union[Pose, None] = None):
        if location is not None is not None:
            super().__init__(node, outcomes=["succeeded", "failed"])
        else:
            super().__init__(
                node, outcomes=["succeeded", "failed"], input_keys=["location"]
            )

        self.navigator = BasicNavigator()
        self.location = location

    def execute(self, userdata):
        if self.location:
            goal_pose = self.location
        elif "location" in userdata:
            goal_pose = userdata.location
        else:
            return "failed"

        goal_stamped = PoseStamped(pose=goal_pose, header=Header(frame_id="map"))

        self.navigator.goToPose(goal_stamped)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self.navigator)

        return (
            "succeeded"
            if self.navigator.getResult() == TaskResult.SUCCEEDED
            else "failed"
        )
