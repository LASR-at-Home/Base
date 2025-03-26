#!/usr/bin/env python3
from typing import Union
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import smach
from geometry_msgs.msg import Pose, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from std_msgs.msg import Header
from .go_to_location import GoToLocation



class TestStateMachine(smach.StateMachine, Node):
    def __init__(
            self, 
            start_pose : Pose, 
            end_pose : Pose, 
    ):
        Node.__init__(self,"test_state_machine")
        smach.StateMachine.__init__(
            self, 
            outcomes=["succeeded","aborted","timed_out"],
            
        )

        with self: 
            smach.StateMachine.add(
                "GoToStart",
                GoToLocation(location=start_pose),
                transitions={"succeeded":"GoBack", "failed":"aborted"},
            )
            smach.StateMachine.add(
                "GoBack",
                GoToLocation(location=end_pose),
                transitions={"succeeded":"GoToStart","failed":"aborted"}

            )






# Go to Location? 




# Do YOLO? 