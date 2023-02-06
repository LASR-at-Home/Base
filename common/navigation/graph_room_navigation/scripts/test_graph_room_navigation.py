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


add_room("1", Point(-1.61784005165, -6.01485300064, 0.003173828125), Point(7.32377958298, -3.92606759071, 0.00618362426758))
add_room("2", Point(-3.93185663223, -5.84905290604, 0.00711822509766), Point(-2.14899969101, -3.93605685234, 0.00291776657104))
add_room("3", Point(-4.03997564316, -3.58257627487, 0.00448608398438), Point(-2.01879358292, -0.0398881435394, 0.00237274169922))
add_room("4", Point(-4.04962348938, 0.282939434052, 0.0023250579834), Point(-2.14625120163, 4.13284730911, 0.00645446777344))
add_room("5", Point(-4.03492736816, 4.57574367523, 0.0018367767334), Point(-2.01340484619, 7.56172800064, 0.00513648986816))
add_room("6", Point(-3.96694850922, 8.09547901154, 0.00531196594238), Point(7.16684436798, 10.0858697891, 0.00330352783203))
add_room("7", Point(4.66051578522, 5.61562347412, 0.00441551208496), Point(7.3139629364, 7.91275978088, 0.00715255737305))
add_room("8", Point(4.62594032288, 0.101803421974, 0.00494575500488), Point(7.11309337616, 5.29711961746, 0.00454330444336))
add_room("9", Point(4.53174972534, -3.38216137886, 0.00150299072266), Point(7.10823154449, -0.169175356627, 0.00512313842773))
add_room("10", Point(-1.46323537827, -3.14745187759, 0.00832939147949), Point(4.02719116211, 7.64226818085, -0.000827789306641))

add_doorway("1", Point(-1.2186807394, -4.94984722137, 0.00253105163574), "2", Point(-2.49440407753, -5.02355384827, 0.00259399414062))
add_doorway("2", Point(-3.14921522141, -4.13905334473, 0.00275802612305), "3", Point(-3.10885477066, -2.93200516701, -0.000947952270508))
add_doorway("3", Point(-3.04880666733, -0.489785671234, 0.00375366210938), "4", Point(-3.02961778641, 0.723525524139, 0.00114250183105))
add_doorway("4", Point(-3.06859636307, 3.93108177185, -0.00118255615234), "5", Point(-2.99962377548, 5.29235267639, -0.000438690185547))
add_doorway("5", Point(-3.00463676453, 7.39667701721, 0.00086784362793), "6", Point(-2.96385860443, 8.67026519775, 0.00597763061523))

add_doorway("1", Point(2.03408622742, -4.09211921692,0.00350570678711 ), "10", Point(1.8931568861,-2.68043470383, 0.000261306762695 ))
add_doorway("10", Point(1.58423185349, 7.51083374023, 0.00188827514648), "6", Point(1.48598980904, 8.66058731079, 0.00163841247559))

add_doorway("1", Point(5.83453989029, -3.95340919495, 0.00461387634277), "9", Point(5.83007049561, -2.79803729057, 0.000888824462891))
add_doorway("9", Point(5.90866088867, -0.641026496887, 0.00300788879395), "8", Point(5.84857082367, 0.735258579254, 0.00522994995117))
add_doorway("8", Point(6.0185213089, 4.96745109558, 0.0035343170166), "7", Point(6.00732564926, 6.25949573517, -0.000198364257812))
add_doorway("7", Point(5.92965555191, 7.66893577576, 0.00309562683105), "6", Point(5.90539455414, 8.90781402588, 0.00558090209961))

plan = plan_to_room("9")
print(plan)


for p1, p2 in zip(plan.points, plan.points[1:]):
    goal = MoveToGoalGoal()
    goal.start_pose.header.stamp = rospy.get_rostime()
    goal.start_pose.header.frame_id = 'map'
    goal.start_pose.pose.position = p1

    goal.end_pose.header.stamp = rospy.get_rostime()
    goal.end_pose.header.frame_id = 'map'
    goal.end_pose.pose.position = p2

    # goal = MoveToGoalAction(
    #     PoseStamped(
    #         Header(
    #             rospy.get_rostime(),
    #             'map',
    #         ),
    #         p1
    #     ),
    #     PoseStamped(
    #         Header(
    #             rospy.get_rostime(),
    #             'map',
    #         ),
    #         p2
    #     )
    # )
    move_to_goal_client.send_goal(goal)
    move_to_goal_client.wait_for_result()