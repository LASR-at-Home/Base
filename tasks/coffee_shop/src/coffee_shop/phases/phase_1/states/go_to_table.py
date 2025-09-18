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
        (label, rospy.get_param(f"/coffee_shop/tables/{label}"))
        for label, tbl in self.context.tables.items()
        if tbl["status"] == "unvisited"
    ]

        if not unvisited:   
            self.context.say("No unvisited tables.")
            return "skip"

        def goal_xy(tbl):
            self.context.say("Go to table getting goal xy")
            pos = tbl.get("approach_pose", {}).get("position", tbl["location"]["position"])
            rospy.loginfo(f"DEBUG: Goal position is {pos}")
            self.context.say("Got the table position")
            return pos["x"], pos["y"]

        label, next_table = min(
            unvisited,
            key=lambda t: np.hypot(goal_xy(t[1])[0] - robot_x, goal_xy(t[1])[1] - robot_y),
        )

        self.context.say(f"I am going to {label}")
        '''
        position, orientation = (
            next_table["location"]["position"],
            next_table["location"]["orientation"],
        )
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.pose = Pose(
            position=Point(**position), orientation=Quaternion(**orientation)
        )
        '''
        # approach_pose; fallback to saved location
        approach_pos = next_table.get("approach_pose", {}).get("position") or next_table["location"]["position"]
        approach_ori = next_table.get("approach_pose", {}).get("orientation") or next_table["location"]["orientation"]

        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.pose = Pose(
            position=Point(**approach_pos), orientation=Quaternion(**approach_ori)
        )

        self.context.move_base_client.send_goal_and_wait(move_base_goal)
        self.context.tables[label]["status"] = "currently visiting"
        self.context.current_table = label
        return "done"
