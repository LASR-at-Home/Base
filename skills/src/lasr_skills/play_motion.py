import smach_ros

from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

from typing import Union


class PlayMotion(smach_ros.SimpleActionState):
    def __init__(self, motion_name: Union[str, None] = None):
        if motion_name is not None:
            super(PlayMotion, self).__init__(
                "play_motion",
                PlayMotionAction,
                goal=PlayMotionGoal(motion_name=motion_name, skip_planning=False),
            )
        else:
            super(PlayMotion, self).__init__(
                "play_motion",
                PlayMotionAction,
                goal_cb=lambda ud, _: PlayMotionGoal(
                    motion_name=ud.motion_name, skip_planning=False
                ),
                input_keys=["motion_name"],
            )
