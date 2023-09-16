#!/usr/bin/env python3
import smach
from tiago_controllers.base_planner import make_plan
from geometry_msgs.msg import Pose, Point


class GoCloserToPerson(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['done'])
        self.context = context

    def execute(self, userdata):
        self.context.voice_controller.async_tts("I think there is a customer waiting. I will go and investigate.")
        pose = self.context.new_customer_pose
        robot_x, robot_y, quaternion = self.context.base_controller.get_current_pose()
        success, point, points = make_plan(Pose(
                position=Point(robot_x, robot_y, 0.0),
                orientation=quaternion
            ), pose[0], pose[1], tol=0.5)
        if success:
            chosen_poses = points[:int(len(points)/2) + 1][::-1]
            for chosen_pose in chosen_poses:
                if self.context.base_controller.sync_to_pose(chosen_pose.pose):
                    break
        self.context.base_controller.sync_face_to(pose[0], pose[1])
        return 'done'