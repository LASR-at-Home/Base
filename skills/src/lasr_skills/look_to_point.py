from smach_ros import RosState, SimpleActionState
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient

from control_msgs.action import PointHead
from geometry_msgs.msg import Point, PointStamped, Vector3

from typing import Union

#TODO: Action completes but doesn't end and move on
class LookToPoint(SimpleActionState):
    def __init__(self, node, pointstamped: Union[None, PointStamped] = None, exec_timeout_sec: float = 5.0):
        super().__init__(node=node, 
                         action_name='/head_controller/point_head_action', 
                         action_spec=PointHead, 
                         goal_cb=self._create_goal, 
                         result_cb=self._result_handle, 
                         input_keys=["pointstamped"] if pointstamped is None else [],
                         exec_timeout=Duration(seconds=exec_timeout_sec))         
        self._pointstamped = pointstamped
        self.node.get_logger().info("LookToPoint - Created State.")
    
    def _create_goal(self, ud, goal):
        goal.pointing_frame = "head_2_link"
        goal.pointing_axis = Vector3(x=1.0, y=0.0, z=0.0)
        goal.max_velocity = 1.0
        goal.target = (
                self._pointstamped
                if self._pointstamped is not None
                else ud.pointstamped
            )
        
        self.node.get_logger().info(f"Sending goal - PointHead to {goal.target}")
        return goal
    
    def _result_handle(self, ud, result_status, result):
        self.node.get_logger().info(
            f"PointHead received result with status: {result_status}"
        )