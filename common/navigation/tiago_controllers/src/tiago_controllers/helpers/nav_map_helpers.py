#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction
from visualization_msgs.msg import Marker
from markers.markers_helpers import create_point_marker
from common_math.math_ import euclidian_distance

def get_dist_to_door(is_robot, x=None, y=None):
    if is_robot:
        robot_pose = rospy.wait_for_message("/robot_pose", PoseWithCovarianceStamped)
        print(f"robot pose: {robot_pose}")
        r = (robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y)
    else:
        r = (x, y)

    door_position = get_pose_from_param("/door/pose")
    print(f"door pose: {door_position}")
    d = (door_position.position.x, door_position.position.y)

    dist = euclidian_distance(r, d)
    print(f"distance to door: {dist}")
    return dist
def clear_costmap():
    """
        Clears costmap using clear_octomap server
    """

    rospy.loginfo('waiting for clear_costmap')
    rospy.wait_for_service('/move_base/clear_costmaps', timeout=10)
    try:
        _clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        response = _clear_costmap()
        rospy.loginfo('clearing costmap done!')
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def how_close_to_object(object_name='/door/pose'):
    """
    Checks how close the robot is to the object stored in param
    Parameters
    ----------
    object_name
    min_dist

    Returns
    -------

    """
    robot_pose = rospy.wait_for_message("/robot_pose", PoseWithCovarianceStamped)
    door_position = get_pose_from_param(object_name)
    r = (robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y)
    d = (door_position.position.x, door_position.position.y)
    dist = euclidian_distance(r, d)
    return dist
def is_close_to_object(object_name='/door/pose', min_dist=0.5):
    """
    Checks if the robot is close to the object stored in param compared to the min_dist
    Parameters
    ----------
    object_name
    min_dist

    Returns
    -------

    """
    dist = how_close_to_object(object_name)
    return dist < min_dist
def get_current_robot_pose():
        robot_pose = rospy.wait_for_message("/robot_pose", PoseWithCovarianceStamped)
        return robot_pose.pose.pose
def get_random_rgb():
    import random
    r = random.randint(0, 255)
    g = random.randint(0, 255)
    b = random.randint(0, 255)
    print(f"Random RGB: ({r}, {g}, {b})")
    return (r, g, b), "This is {}, {}, {}".format(r, g, b)
def rank(points_name="lift/centers"):
    centers = rospy.get_param(points_name)
    print("centers")
    print(centers)

    # get the distance of each center to the door
    distances = []
    for center in centers:
        distances.append((get_dist_to_door(False, center[0], center[1]), center))

    # rank the distances (dist, center-person) and add the robot
    distances.append((get_dist_to_door(True), (0, 0)))
    print("distances")
    print(distances)

    # sort the distances
    sorted_distances = sorted(distances, key=lambda x: x[0])
    print("sorted distances")
    print(sorted_distances)
    # random_colour, random_text = get_random_rgb()
    people_pose_pub = rospy.Publisher("/people_poses", Marker, queue_size=100)
    for i, dist in enumerate(sorted_distances):
        mk = create_point_marker(dist[1][0], dist[1][1], 0, i)
        people_pose_pub.publish(mk)
        rospy.sleep(2)

    # mk = self.create_point_marker(sorted_distances[0][1][0], sorted_distances[0][1][1], 0, 0, random_colour, random_text)
    # people_pose_pub.publish(mk)
    # oif the robot is closest or second to closest return true
    if sorted_distances[0][1] == (0, 0) or sorted_distances[1][1] == (0, 0):
        return True
    else:
        return False
def counter(topic="/counter_lift/counter", count_default=3):
    count = rospy.get_param(topic)
    rospy.loginfo("count: " + str(topic) + "---> " + str(count))

    if int(count) > count_default:
        return "counter"

    # set the param to count how many times it has failed in this state
    count += 1
    rospy.set_param(topic, count)

def play_motion_goal(client, motion_name='home'):
        client.wait_for_server()
        rospy.sleep(0.5)
        goal = PlayMotionGoal()

        goal.motion_name = motion_name
        client.send_goal(goal)
        result = client.wait_for_result()
        state = client.get_state()
        return result, state