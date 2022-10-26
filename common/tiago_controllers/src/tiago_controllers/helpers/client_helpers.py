#!/usr/bin/env python3

from actionlib_msgs.msg import GoalStatus


def is_running(client):
    return client.get_state() == GoalStatus.PENDING or client.get_state() == GoalStatus.ACTIVE


def is_terminated(client):
    return client.get_state() == GoalStatus.LOST or client.get_state() == GoalStatus.PREEMPTED or \
           client.get_state() == GoalStatus.ABORTED
