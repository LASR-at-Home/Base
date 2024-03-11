import smach
import smach_ros

from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from std_srvs.srv import Empty

from lasr_skills import Say, ListenFor


class HandoverObject(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(
            self, outcomes=["succeeded", "failed"], input_keys=["object_name"]
        )

        with self:
            smach.StateMachine.add(
                "CLEAR_OCTOMAP",
                smach_ros.ServiceState("clear_octomap", Empty),
                transitions={"succeeded": "RECEIVE_OBJECT", "aborted": "failed"},
            )
            smach.StateMachine.add(
                "HANDOVER_OBJECT",
                smach_ros.SimpleActionState(
                    "play_motion",
                    PlayMotionAction,
                    goal=PlayMotionGoal(
                        motion_name="handover_object", skip_planning=False
                    ),
                ),
                transitions={"succeeded": "succeeded", "aborted": "failed"},
            )
            smach.StateMachine.add(
                "SAY_TAKE",
                Say(
                    format_str="Please take the {} from my end-effector, and say done.",
                ),
                transitions={"succeeded": "succeeded", "aborted": "failed"},
                remapping={"placeholders": "object_name"},
            )
            smach.StateMachine.add(
                "LISTEN_DONE",
                ListenFor("done"),
                transitions={
                    "succeeded": "OPEN_GRIPPER",
                    "not_done": "LISTEN_DONE",
                    "failed": "failed",
                },
            )
            smach.StateMachine.add(
                "OPEN_GRIPPER",
                smach_ros.SimpleActionState(
                    "play_motion",
                    PlayMotionAction,
                    goal=PlayMotionGoal(motion_name="open_gripper", skip_planning=True),
                ),
                transitions={"succeeded": "succeeded", "aborted": "failed"},
            )
            smach.StateMachine.add(
                "FOLD_ARM",
                smach_ros.SimpleActionState(
                    "play_motion",
                    PlayMotionAction,
                    goal=PlayMotionGoal(motion_name="fold_arm", skip_planning=False),
                ),
                transitions={"succeeded": "succeeded", "aborted": "failed"},
            )
