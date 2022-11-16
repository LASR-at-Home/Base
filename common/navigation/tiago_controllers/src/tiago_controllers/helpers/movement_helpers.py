#!/usr/bin/env python3
import rospy
from tiago_controllers.helpers import get_pose_from_param, is_running
from models.controllers import Controllers
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

HEAD_TIME = 1.5
TORSO_LOW = 0.18
TORSO_HIGH = 0.2
HEAD_TWIST = 0.5


def _get_path() -> Path:
    return rospy.wait_for_message("/move_base/GlobalPlanner/plan", Path)


def _get_halfway() -> PoseStamped:
    path = _get_path().poses
    if not path or len(path) < 3:
        return PoseStamped()
    else:
        return path[int(len(path) / 2)]

def _is_point_in_rect(bl, tr, p):
    return bl[0] < p[0] < tr[0] and bl[1] < p[1] < tr[1]

def _is_halfway(base_controller) -> bool:
    pose = _get_halfway()
    if pose == PoseStamped():
        return True
    else:
        tol = 1.0
        x, y = pose.pose.position.x, pose.pose.position.y
        return _is_point_in_rect([x - tol, y - tol], [x + tol, y + tol], base_controller.get_pose())

def move_to_position(is_sync, position, base_controller):
    is_real_life = rospy.get_published_topics(namespace='/pal_head_manager')
    rospy.sleep(2)
    if is_real_life and is_sync:
        print("is real life")
        base_controller.sync_to_pose(get_pose_from_param(position))
    elif not is_real_life and is_sync:
        place = position + "_simulation"
        base_controller.sync_to_pose(get_pose_from_param(place))
    elif not is_sync and is_real_life:
        print("is real life")
        base_controller.async_to_pose(get_pose_from_param(position))
    elif not is_sync and not is_real_life:
        place = position + "_simulation"
        base_controller.async_to_pose(get_pose_from_param(place))

def move_breath_and_search(controllers: Controllers, position: str):
    base_controller = controllers.base_controller
    move_to_position(is_sync=False, position=position, base_controller=base_controller)
    torso_controller = controllers.torso_controller
    head_controller = controllers.head_controller
    is_head_right = False
    is_torso_up = False
    is_halfway = False
    while not rospy.is_shutdown() and is_running(base_controller.get_client()):
        if not is_halfway:
            if _is_halfway(base_controller):
                is_halfway = True
            elif not is_running(head_controller.get_client()):
                if is_head_right:
                    head_controller.async_reach_to(HEAD_TWIST, -1, time_from_start=HEAD_TIME)
                else:
                    head_controller.async_reach_to(-1 * HEAD_TWIST, -1, time_from_start=HEAD_TIME)
                is_head_right = not is_head_right
                rospy.sleep(0.1)
        if not is_running(torso_controller.get_client()):
            torso_controller.async_reach_to(TORSO_LOW) if is_torso_up else torso_controller.async_reach_to(TORSO_HIGH)
            is_torso_up = not is_torso_up
            rospy.sleep(0.1)
    torso_controller.async_reach_to(TORSO_HIGH)
