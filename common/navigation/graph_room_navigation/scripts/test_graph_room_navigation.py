#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from std_msgs.msg import Header
from graph_room_navigation.srv import AddRoom, AddCrossing, PlanToPoint, PlanToRoom
from tiago_controllers import BaseController
import actionlib
from unsafe_traversal.msg import MoveToGoalAction, MoveToGoalGoal
from lasr_voice.voice import Voice

rospy.init_node("test_graph_room_navigation")
add_room = rospy.ServiceProxy("graph_navigation_server/add_room", AddRoom)
add_doorway = rospy.ServiceProxy("graph_navigation_server/add_doorway", AddCrossing)
plan_to_room = rospy.ServiceProxy("graph_navigation_server/plan_to_room", PlanToRoom)
plan_to_pose = rospy.ServiceProxy("graph_navigation_server/plan_to_point", PlanToPoint)
bc = BaseController()

# setup action client
move_to_goal_client = actionlib.SimpleActionClient(
    '/unsafe_traversal/move_to_goal', MoveToGoalAction)
move_to_goal_client.wait_for_server()


# add_room("1", Point(-0.35, -3.68, 0.0), Point(4.03, 3.23, 0.0))
add_room("1", Point(-4.09056949615, 7.93803453445, 0.0), Point(6.9501953125, 10.0224742889, 0.0))
add_room("2", Point(-1.75154161453, -3.51443386078, 0.0), Point(4.0252828598, 7.61349916458, 0.0))

add_doorway("1", Point(1.51952576637, 9.26592350006, 0.0), "2", Point(1.56058132648, 7.16878604889, 0.0))

# add_room("2", Point(1.79, 3.24, 0.0), Point(5.02, 7.77, 0.0))
# add_room("3", Point(-0.65, 6.53, 0.0), Point(12.32, 29.9, 0.0))
# add_room("4", Point(2.36436057091, 28.147977829, 0.0), Point(6.12658166885, 31.6530075073, 0.0))

# add_doorway("1", Point(2.63982193767, 2.27391396633, 0.0), "2", Point(2.33113836267, 3.36879451667, 0.0))
# add_doorway("2", Point(4.62, 7.01, 0.0), "3", Point(4.11, 8.37, 0.0))
# add_doorway("3", Point(5.14, 27.38, 0.0), "4", Point(4.75, 29.71, 0.0))

plan = plan_to_room("1")
print(plan)
asked_for_help = False
tries = 0
max_tries = 3

phrases = [
    "Could someone please open this door for me?",
    "Open sesame",
    "This door is not going to open itself!"
]

for p1, p2 in zip(plan.points[0::2], plan.points[1::2]):

    result = False
    tries = 0
    while not result:
        goal = MoveToGoalGoal()
        goal.start_pose.header.stamp = rospy.get_rostime()
        goal.start_pose.header.frame_id = 'map'
        goal.start_pose.pose.position = p1

        goal.end_pose.header.stamp = rospy.get_rostime()
        goal.end_pose.header.frame_id = 'map'
        goal.end_pose.pose.position = p2
        move_to_goal_client.send_goal(goal)

        move_to_goal_client.wait_for_result()
        result = move_to_goal_client.get_result().success

        if not result:
            if not asked_for_help and tries < max_tries:
                print(phrases[tries])
                asked_for_help = True
                tries+=1
            elif asked_for_help and tries < max_tries:
                asked_for_help = False
            elif tries >= max_tries:
                print("I give up...")
                exit(0)
            rospy.sleep(10)
