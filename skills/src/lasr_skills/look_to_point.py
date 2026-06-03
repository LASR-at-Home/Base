import yasmin
import yasmin_ros
import rclpy
from control_msgs.action import PointHead
from geometry_msgs.msg import Point, PointStamped, Vector3
from std_msgs.msg import Header

from typing import Union



class LookToPoint(yasmin_ros.ActionState):
    def __init__(
        self,
        pointstamped: Union[None, PointStamped] = None,
    ):
        
        super().__init__(
            action_name="/head_controller/point_head_action",
            action_type=PointHead,
            create_goal_handler=self._create_goal,
            response_timeout=10.0,
            result_handler=self._result_handle,
        )
        if pointstamped is None:
            self.add_input_key('pointstamped')
        self._pointstamped = pointstamped

    def _create_goal(self, blackboard):
        target = (
            self._pointstamped
            if self._pointstamped is not None
            else blackboard['pointstamped']
        )

        goal = PointHead.Goal()
        goal.pointing_frame = "head_2_link"
        goal.pointing_axis = Vector3(x=1.0, y=0.0, z=0.0)
        goal.max_velocity = 1.0
        goal.target = target

        yasmin.YASMIN_LOG_INFO(
            "Sending PointHead goal: "
            f"frame={goal.target.header.frame_id}, "
            f"point=({goal.target.point.x:.3f}, "
            f"{goal.target.point.y:.3f}, {goal.target.point.z:.3f})"
        )

        return goal
    
    def _result_handle(self, blackboard, response):
        self._node.get_logger().info(f"Received result with response: {response}")
        if response == 'timeout':
            return 'failed'
        return 'succeeded'
    

def main():
    rclpy.init()
    
    yasmin_ros.set_ros_loggers()
    
    sm = yasmin.StateMachine(outcomes=['succeeded', 'failed'], handle_sigint=True)
    
    sm.add_state('LOOK', LookToPoint(pointstamped=PointStamped(header=Header(frame_id="base_link"), point=Point(x=1.2512260675430298, y=-1.399413824081421, z=-0.094940185546875))), transitions={'succeeded': 'succeeded', 'aborted': 'failed', 'canceled': 'failed'})
    
    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(f"State machine finished with outcome {outcome}")
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)
        
    if rclpy.ok():
        rclpy.shutdown()
            
