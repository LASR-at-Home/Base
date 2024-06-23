import smach_ros
import rospy

from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

from typing import Union, List


class PlayMotion(smach_ros.SimpleActionState):
    def _needs_planning(self, motion_name: str) -> bool:
        joints: List[str] = rospy.get_param(
            f"/play_motion/motions/{motion_name}/joints"
        )
        needs_planning: bool = any(
            "arm" in joint or "gripper" in joint for joint in joints
        )

        print(f"Motion {motion_name} needs planning: {needs_planning}")

        return needs_planning

    def __init__(self, motion_name: Union[str, None] = None):
        # TODO: the play motion action server is always returning 'aborted', figure out what's going on
        if motion_name is not None:
            super(PlayMotion, self).__init__(
                "play_motion",
                PlayMotionAction,
                goal=PlayMotionGoal(
                    motion_name=motion_name,
                    skip_planning=not self._needs_planning(motion_name),
                ),
                result_cb=lambda _, __, ___: "succeeded",
            )
        else:
            super(PlayMotion, self).__init__(
                "play_motion",
                PlayMotionAction,
                goal_cb=lambda ud, _: PlayMotionGoal(
                    motion_name=ud.motion_name,
                    skip_planning=not self._needs_planning(ud.motion_name),
                ),
                input_keys=["motion_name"],
                result_cb=lambda _, __, ___: "succeeded",
            )
