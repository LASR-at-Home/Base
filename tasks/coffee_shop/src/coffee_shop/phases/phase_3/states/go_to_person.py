import smach
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
import rospy
from move_base_msgs.msg import MoveBaseGoal
import numpy as np
import math
from scipy.spatial.transform import Rotation as R


class GoToPerson(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=["done"])
        self.context = context

    def execute(self, userdata):
        location = rospy.get_param("/coffee_shop/wait/approach2")
        position, orientation = location["position"], location["orientation"]
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.pose = Pose(
            position=Point(**position), orientation=Quaternion(**orientation)
        )
        self.context.move_base_client.send_goal_and_wait(move_base_goal)
        if self.context.new_customer_pose:
            robot_pose = rospy.wait_for_message(
                "/amcl_pose", PoseWithCovarianceStamped
            ).pose.pose
            robot_x, robot_y = robot_pose.position.x, robot_pose.position.y
            customer_x, customer_y = (
                self.context.new_customer_pose.point.x,
                self.context.new_customer_pose.point.y,
            )
            dx, dy = customer_x - robot_x, customer_y - robot_y
            theta_deg = np.degrees(math.atan2(dy, dx))
            quaternion = R.from_euler("z", theta_deg, degrees=True).as_quat()
            move_base_goal = MoveBaseGoal()
            move_base_goal.target_pose.header.frame_id = "map"
            move_base_goal.target_pose.pose = Pose(
                position=Point(**position),
                orientation=Quaternion(*quaternion),
            )
            self.context.move_base_client.send_goal_and_wait(move_base_goal)

        return "done"
