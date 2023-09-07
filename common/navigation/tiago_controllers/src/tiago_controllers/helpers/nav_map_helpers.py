#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped
from tiago_controllers.helpers.pose_helpers import get_pose_from_param
from common_math.math_ import euclidian_distance
from play_motion_msgs.msg import PlayMotionGoal, PlayMotionAction


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
    r = (robot_pose.position.x, robot_pose.position.y)
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
def counter(topic="/counter_lift/counter"):
    count = rospy.get_param(topic)
    rospy.loginfo("count: " + str(topic) + "---> " + str(count))

    if int(count) > 3:
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