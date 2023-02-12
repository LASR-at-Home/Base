#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from std_msgs.msg import Header
from graph_room_navigation.srv import AddRoom, AddCrossing, PlanToPose, PlanToRoom
from tiago_controllers import BaseController
import actionlib
from unsafe_traversal.msg import MoveToGoalAction, MoveToGoalGoal

rospy.init_node("test_graph_room_navigation")
add_room = rospy.ServiceProxy("graph_navigation_server/add_room", AddRoom)
add_doorway = rospy.ServiceProxy("graph_navigation_server/add_doorway", AddCrossing)
plan_to_room = rospy.ServiceProxy("graph_navigation_server/plan_to_room", PlanToRoom)
plan_to_pose = rospy.ServiceProxy("graph_navigation_server/plan_to_pose", PlanToPose)

# setup action client
move_to_goal_client = actionlib.SimpleActionClient(
    '/unsafe_traversal/move_to_goal', MoveToGoalAction)
move_to_goal_client.wait_for_server()


add_room("1", Point(-0.35, -3.68, 0.0), Point(4.03, 3.23, 0.0))
add_room("2", Point(1.79, 3.24, 0.0), Point(5.02, 7.77, 0.0))
add_room("3", Point(-0.65, 6.53, 0.0), Point(12.32, 29.9, 0.0))
add_room("4", Point(-5.97, 26.50, 0.0), Point(6.58, 31.87, 0.0))

add_doorway("1", Point(2.65, 2.52, 0.0), "2", Point(2.40, 2.46, 0.0))
add_doorway("2", Point(4.62, 7.01, 0.0), "3", Point(4.11, 8.37, 0.0))
add_doorway("3", Point(5.14, 27.38, 0.0), "4", Point(4.75, 29.71))

plan = plan_to_room("4")

for p1, p2 in zip(plan.points, plan.points[1:]):
    goal = MoveToGoalGoal()
    goal.start_pose.header.stamp = rospy.get_rostime()
    goal.start_pose.header.frame_id = 'map'
    goal.start_pose.pose.position = p1

    goal.end_pose.header.stamp = rospy.get_rostime()
    goal.end_pose.header.frame_id = 'map'
    goal.end_pose.pose.position = p2
    move_to_goal_client.send_goal(goal)
    move_to_goal_client.wait_for_result()