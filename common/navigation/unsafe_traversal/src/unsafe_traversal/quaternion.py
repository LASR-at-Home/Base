#!/usr/bin/env python3
import math


def align_poses(start_pose, end_pose):
    '''
    Align both poses to point forwards from start to end pose
    This can probably be done using TF but that complicates things
    '''

    if start_pose.header.frame_id != end_pose.header.frame_id:
        raise Exception("frame_id of start and end pose does not match")

    angle = math.atan2(
        end_pose.pose.position.y - start_pose.pose.position.y,
        end_pose.pose.position.x - start_pose.pose.position.x
    )

    z = math.sin(angle / 2)
    w = math.cos(angle / 2)

    start_pose.pose.orientation.x = 0
    start_pose.pose.orientation.y = 0
    start_pose.pose.orientation.z = z
    start_pose.pose.orientation.w = w

    end_pose.pose.orientation.x = 0
    end_pose.pose.orientation.y = 0
    end_pose.pose.orientation.z = z
    end_pose.pose.orientation.w = w
