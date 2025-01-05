import smach_ros
from rclpy.node import Node
from play_motion2_msgs.action import PlayMotion2
# https://github.com/pal-robotics/play_motion2

from typing import Union, List


# TODO: test initialisation of states; check that PlayMotion2 is found

class PlayMotion(smach_ros.SimpleActionState, Node):
    def _needs_planning(self, motion_name: str) -> bool:
        joints_param = self.get_parameter(
            f"/play_motion2/motions/{motion_name}/joints"
        )
        joints: List[str] = joints_param.value
        needs_planning: bool = any(
            "arm" in joint or "gripper" in joint for joint in joints
        )

        print(f"Motion {motion_name} needs planning: {needs_planning}")

        return needs_planning

    def __init__(self, motion_name: Union[str, None] = None):
        Node.__init__(self, "play_motion")
        # TODO: the play motion action server is always returning 'aborted', figure out what's going on
        #  This is an issue from ROS1, check if it's been resolved in ROS2
        if motion_name is not None:
            smach_ros.SimpleActionState(self, "play_motion", PlayMotion2).__init__(
                PlayMotion,
                "play_motion",
                PlayMotion2,
                goal=PlayMotion2.Goal(
                    motion_name=motion_name,
                    skip_planning=not self._needs_planning(motion_name),
                ),
                result_cb=lambda _, __, ___: "succeeded",
            )
        else:
            smach_ros.SimpleActionState(self, "play_motion", PlayMotion2).__init__(
                PlayMotion,
                "play_motion",
                PlayMotion2,
                goal_cb=lambda ud, _: PlayMotion2.Goal(
                    motion_name=ud.motion_name,
                    skip_planning=not self._needs_planning(ud.motion_name),
                ),
                input_keys=["motion_name"],
                result_cb=lambda _, __, ___: "succeeded",
            )
