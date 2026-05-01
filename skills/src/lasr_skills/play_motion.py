import smach_ros

from rclpy.node import Node

from play_motion2_msgs.action import PlayMotion2

# https://github.com/pal-robotics/play_motion2

from typing import Union, List

# TODO: test initialisation of states; check that PlayMotion2 is found


class PlayMotion(smach_ros.SimpleActionState):
    @staticmethod
    def _needs_planning(node: Node, motion_name: str) -> bool:
        joint_param: str = f"motions.{motion_name}.joints"
        if not node.has_parameter(joint_param):
            node.declare_parameter(joint_param, [""])
        joints: List[str] = node.get_parameter(joint_param).value
        needs_planning: bool = any(
            "arm" in joint or "gripper" in joint for joint in joints
        )

        print(f"Motion {motion_name} needs planning: {needs_planning}")

        return needs_planning

    def __init__(self, node, motion_name: Union[str, None] = None):
        # TODO: the play motion action server is always returning 'aborted', figure out what's going on
        #  This is an issue from ROS1, check if it's been resolved in ROS2
        # TODO: (From BEN) I think the previous code is wrong?
        if motion_name is not None:
            super().__init__(
                node,
                "play_motion",
                PlayMotion2,
                goal=PlayMotion2.Goal(
                    motion_name=motion_name,
                    skip_planning=not self._needs_planning(node, motion_name),
                ),
                result_cb=lambda _, __, ___: "succeeded",
            )
        else:
            super().__init__(
                node,
                "play_motion",
                PlayMotion2,
                goal_cb=lambda ud, _: PlayMotion2.Goal(
                    motion_name=ud.motion_name,
                    skip_planning=not self._needs_planning(node, ud.motion_name),
                ),
                input_keys=["motion_name"],
                result_cb=lambda _, __, ___: "succeeded",
            )
