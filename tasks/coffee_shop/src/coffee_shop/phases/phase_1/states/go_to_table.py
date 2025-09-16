import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal
import numpy as np


class GoToTable(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=["done"])
        self.context = context

    def execute(self, userdata):
        robot_pose = rospy.wait_for_message(
            "/amcl_pose", PoseWithCovarianceStamped
        ).pose.pose
        robot_x, robot_y = robot_pose.position.x, robot_pose.position.y
        unvisited = [
            (label, rospy.get_param(f"/tables/{label}"))
            for label, table in self.context.tables.items()
            if table["status"] == "unvisited"
        ]
        closest_table = min(
            unvisited,
            key=lambda table: np.linalg.norm(
                [
                    table[1]["location"]["position"]["x"] - robot_x,
                    table[1]["location"]["position"]["y"] - robot_y,
                ]
            ),
        )
        label, next_table = closest_table
        self.context.say(f"I am going to {label}")
        position, orientation = (
            next_table["location"]["position"],
            next_table["location"]["orientation"],
        )
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.pose = Pose(
            position=Point(**position), orientation=Quaternion(**orientation)
        )
        self.context.move_base_client.send_goal_and_wait(move_base_goal)
        self.context.tables[label]["status"] = "currently visiting"
        self.context.current_table = label
        return "done"
