from typing import Union
import rclpy
import smach
import smach_ros
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from stretch_nav2.robot_navigator import BasicNavigator
from std_msgs.msg import Header
from rclpy.time import Time


class GoToLocation(smach_ros.SimpleActionState, Node):
    def __init__(
        self,
        location: Union[Pose, None] = None,
        location_param: Union[str, None] = None,
    ):
        Node.__init__(self, "go_to_location")
        # todo: not entirely sure for smach state and node init in this way
        if location is not None or location_param is not None:
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
            )
        else:
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["location"],
            )

        self.navigator = BasicNavigator()
        self.location = location
        self.location_param = location_param

    def execute(self, userdata):
        if self.location:
            goal_pose = self.location
        elif self.location_param:
            position_param = self.get_parameter(f"{self.location_param}.position").get_parameter_value().double_array_value
            orientation_param = self.get_parameter(f"{self.location_param}.orientation").get_parameter_value().double_array_value
            goal_pose = Pose(
                position=Point(x=position_param[0], y=position_param[1], z=position_param[2]),
                orientation=Quaternion(x=orientation_param[0], y=orientation_param[1], 
                                    z=orientation_param[2], w=orientation_param[3])
            )
        elif "location" in userdata:
            goal_pose = userdata.location
        else:
            self.navigator.get_logger().error("No goal provided by userdata or from initialisation.")
            return "failed"
        
        # todo: need to check if time is necessary here or not.
        goal_stamped = PoseStamped(pose=goal_pose, header=Header(frame_id="map"), stamp=Time().to_msg())
        
        self.navigator.goToPose(goal_stamped)

        # todo: not sure if needed to loop to wait.
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self.navigator)

        return "succeeded" if self.navigator.getResult() == 2 else "failed"
