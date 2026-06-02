from typing import Union

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus

import yasmin
import yasmin_ros

from play_motion2_msgs.action import PlayMotion2

# https://github.com/pal-robotics/play_motion2


class PlayMotion(yasmin_ros.ActionState):
    def __init__(self, motion_name):
        super().__init__(
            action_name="/play_motion2",
            action_type=PlayMotion2,
            create_goal_handler=self._create_goal,
            result_handler=self._result_handle,
        )
        if motion_name is None:
            self.add_input_key('motion_name')

        self.motion_name = motion_name

    def _create_goal(self, blackboard):
        goal = PlayMotion2.Goal()
        goal.motion_name = (
            blackboard['motion_name'] if self.motion_name is None else self.motion_name
        )
        goal.skip_planning = False

        self._node.get_logger().info(f"GIVING GOAL of {goal.motion_name}")
        
        return goal

    def _result_handle(self, blackboard, response):
        self._node.get_logger().info(f"Received result with response: {response}")
        return 'succeeded'
    

def main():
    rclpy.init()
    
    yasmin_ros.set_ros_loggers()
    
    sm = yasmin.StateMachine(outcomes=['succeeded', 'failed'], handle_sigint=True)
    sm.add_input_key('motion_name')
    sm.add_state('PLAYMOTION', PlayMotion('look_down_right'), transitions={'succeeded': 'succeeded', 'aborted': 'failed', 'canceled': 'failed'})
    
    
    
    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)
        
    if rclpy.ok():
        rclpy.shutdown()
    
        
        
