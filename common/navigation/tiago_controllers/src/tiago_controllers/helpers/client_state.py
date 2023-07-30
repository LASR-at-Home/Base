#!/usr/bin/env python3

from actionlib_msgs.msg import GoalStatus
import rospy
from actionlib_msgs.msg import GoalID


def is_running(client):
    return client.get_state() == GoalStatus.PENDING or client.get_state() == GoalStatus.ACTIVE


def cancel_goal(topic, client):
    try:
        pub = rospy.Publisher(topic, GoalID, queue_size=1)
        goal_id = GoalID()
        pub.publish(goal_id)
        while not (client.get_state() in [GoalStatus.PREEMPTED, GoalStatus.ABORTED, GoalStatus.REJECTED,
                                          GoalStatus.RECALLED, GoalStatus.SUCCEEDED]):
            print("canceling the goal...")
            rospy.sleep(0.5)
    except AttributeError:
        print("you need to input a valid topic in the form of a string")
    except Exception:
        print("you need a valid topic to cancel the goal")

def is_terminated(client):
    return client.get_state() == GoalStatus.LOST or client.get_state() == GoalStatus.PREEMPTED or \
           client.get_state() == GoalStatus.ABORTED