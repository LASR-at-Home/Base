#!/usr/bin/env python3
import rospy
import smach
from tiago_controllers.utils import activate_robot_navigation
from tiago_controllers.controllers import ReachToRadius

class GoTo(smach.State):
    def __init__(self, base_controller: ReachToRadius):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['location', 'prev'],
                             output_keys=['prev'])
        self.base_controller = base_controller

    def execute(self, userdata):
        # activate navigation on the robot
        activate_robot_navigation(True)
        self.goal = userdata.location

        # go to radius if previous state was scanning the room to detect the host
        if userdata.prev == 'Scan':
            rospy.sleep(2.)  # give it a 2 seconds for obstacle aware nav
            success = self.base_controller.sync_to_radius(self.goal.position.x, self.goal.position.y, radius=2.0,
                                                          tol=0.2)

            if success:
                success = self.base_controller.sync_face_to(self.goal.position.x, self.goal.position.y)
        else:
            success = self.base_controller.sync_to_pose(self.goal)

        userdata.prev = 'GoTo'
        if success:
            rospy.loginfo(f"sending pose goal {self.goal.position} to base_controller...")
            return 'succeeded'
        else:
            rospy.loginfo(f"base controller failed to go to position {self.goal.position}")
            return 'failed'
