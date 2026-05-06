from smach_ros import SimpleActionState

from rclpy.node import Node
from rclpy.duration import Duration

from play_motion2_msgs.action import PlayMotion2

# https://github.com/pal-robotics/play_motion2

from typing import Union, List

# TODO: test initialisation of states; check that PlayMotion2 is found


class PlayMotion(SimpleActionState):

    def __init__(self, node, motion_name: Union[str, None] = None):
        # TODO: the play motion action server is always returning 'aborted', figure out what's going on
        #  This is an issue from ROS1, check if it's been resolved in ROS2
        # TODO: (From BEN) I think the previous code is wrong?
        # if motion_name is not None:
        #     super().__init__(
        #         node,
        #         "play_motion2",
        #         PlayMotion2,
        #         goal=PlayMotion2.Goal(
        #             motion_name=motion_name,
        #             skip_planning=True,    # Executor automatically decides - (Change to False)
        #         ),
        #         result_cb=lambda _, __, ___: "succeeded",
        #     )
        # else:
        #     super().__init__(
        #         node,
        #         "play_motion2",
        #         PlayMotion2,
        #         goal_cb=lambda ud, _: PlayMotion2.Goal(
        #             motion_name=ud.motion_name,
        #             skip_planning=True,    # Executor automatically decides
        #         ),
        #         input_keys=["motion_name"],
        #         result_cb=lambda _, __, ___: "succeeded",
        #     )

        super().__init__(
            node=node,
            action_name="/play_motion2",
            action_spec=PlayMotion2,
            goal_cb=self.create_goal,
            result_cb=self.handle_result,
            input_keys=["motion_name"]  if motion_name is None else [],
            # server_wait_timeout=Duration(seconds=5.0),  #TODO: Ensure it works
            exec_timeout=Duration(seconds=5.0),
        )
        
        self.motion_name = motion_name
                         
                         
    def create_goal(self, ud, goal):
        if self.motion_name is None:
            goal.motion_name = ud.motion_name
        else:
            goal.motion_name = self.motion_name

        goal.skip_planning=False

        self.node.get_logger().warn(f"PlayMotion Goal sent: {goal} ")
        return goal
    
    def handle_result(self, ud, status, result):
        self.node.get_logger().warn(f"PlayMotion Result: {result} ")