import smach
import smach_ros

from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from std_srvs.srv import Empty

from lasr_skills import Say, ListenFor


class ReceiveObject(smach.StateMachine):

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
                "RECEIVE_OBJECT",
                smach_ros.SimpleActionState(
                    "play_motion",
                    PlayMotionAction,
                    goal=PlayMotionGoal(
                        motion_name="receive_object", skip_planning=False
                    ),
                ),
                transitions={"succeeded": "succeeded", "aborted": "failed"},
            )
            smach.StateMachine.add(
                "SAY_PLACE",
                Say(
                    format_str="Please place the {} in my end-effector, and say done.",
                ),
                transitions={"succeeded": "succeeded", "aborted": "failed"},
                remapping={"placeholders": "object_name"},
            )
            smach.StateMachine.add(
                "LISTEN_DONE",
                ListenFor("done"),
                transitions={
                    "succeeded": "CLOSE_GRIPPER",
                    "not_done": "LISTEN_DONE",
                    "failed": "failed",
                },
            )
            smach.StateMachine.add(
                "CLOSE_GRIPPER",
                smach_ros.SimpleServiceState(
                    "parallel_gripper_controller/grasp", Empty
                ),
                transitions={"succeeded": "FOLD_ARM", "aborted": "failed"},
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
