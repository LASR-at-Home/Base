#!/usr/bin/env python2
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

add_room("1", Point(-0.117438435555, -4.53656673431, 0.0), Point(6.54642724991, 9.22430896759, 0.0))
add_room("2", Point(-0.149731636047, 8.49707221985, 0.0), Point(12.5422954559, 27.9610614777, 0.0))

add_doorway("1", Point(4.10271644592, 8.13961029053, 0.0), "2", Point(3.62456274033, 10.1467380524, 0.0))


plan = plan_to_room("2")
print(plan)
asked_for_help = False
tries = 0
max_tries = 3

phrases = [
    "Could someone please open this door for me?",
    "Open sesame",
    "This door is not going to open itself!"
]


voice = Voice()
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
                voice.sync_tts(phrases[tries])
                asked_for_help = True
                tries+=1
            elif asked_for_help and tries < max_tries:
                asked_for_help = False
            elif tries >= max_tries:
                voice.sync_tts("I give up...")
                exit(0)
            rospy.sleep(10)
