#!/usr/bin/env python3
import rospy

from tiago_controllers import BaseController
from tiago_controllers.base_planner import make_plan
import smach

from geometry_msgs.msg import Pose, Quaternion, Point
import dynamic_reconfigure.client

from robocup_receptionist.utils import clear_costmap
class EnterArena(smach.State):

    def __init__(self, base_controller, x, y):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.base_controller = base_controller
        self.x = x
        self.y = y

        # self.client = dynamic_reconfigure.client.Client(<node_to_reconfigure>)


    def execute(self, userdata):
        self.base_controller.linear_move(2, .3)  
        clear_costmap()
        rospy.sleep(2)


        print(self.base_controller.sync_to_pose(Pose(position=Point(x, y, 0), orientation=Quaternion(0,0,0,1))))
        return 'succeeded'

        # # failed = True
        # # while failed:
        # success, goal = make_plan(
        #     Pose(
        #         position = Point(robot_x, robot_y, 0),
        #         orientation = Quaternion(0, 0, 0, 1)
        #     ),
        #     self.x, self.y,
        #     max_dist = 10.
        # )
        # # # if not success:
        # # #     return 'failed'
        # # # else:
        
        # self.base_controller.sync_to_pose(goal)
        # return 'succeeded'
        #     return 'succeeded'

class InspectionSM(smach.StateMachine):
    def __init__(self, x, y):
        smach.StateMachine.__init__(self, outcomes=['end'])
        self.base_controller = BaseController()

        with self:
            smach.StateMachine.add(
                'ENTER_ARENA', EnterArena(self.base_controller, x, y), transitions={'succeeded' : 'end', 'failed' : 'ENTER_ARENA'}
            )

if __name__ == "__main__":
    rospy.init_node("inspection_test")
    x = 1.88
    y = -0.1
    sm = InspectionSM(x, y)
    sm.execute()
