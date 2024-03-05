import smach
from smach_ros import SimpleActionState

from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from pal_interaction_msgs.msg import TtsGoal, TtsAction, TtsText

from lasr_skills import ListenWakeWord


class ReceiveObject(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(
            self, outcomes=["succeeded", "failed"], input_keys=["object_name"]
        )

        with self:
            # TODO: clear octomap and look around
            smach.StateMachine.add(
                "RECEIVE_OBJECT",
                SimpleActionState(
                    "play_motion",
                    PlayMotionAction,
                    goal=PlayMotionGoal(
                        motion_name="receive_object", skip_planning=False
                    ),
                ),
                transitions={"succeeded": "succeeded", "aborted": "failed"},
            )
            smach.StateMachine.add(
                "SAY",
                SimpleActionState(
                    "tts",
                    TtsAction,
                    goal_cb=lambda ud, _: TtsGoal(
                        rawtext=TtsText(
                            text=f"Please place the {ud.object_name} in my hand, and say 'done' when you are ready.",
                            lang_id="en_GB",
                        )
                    ),
                    input_keys=["object_name"],
                ),
            )
            smach.StateMachine.add(
                "WAIT_FOR_SPEECH",
                ListenWakeWord("done"),
                transitions={"succeeded": "CLOSE_GRIPPER", "failed": "WAIT_FOR_SPEECH"},
            )

            smach.StateMachine.add(
                "CLOSE_GRIPPER",
                SimpleActionState(
                    "play_motion",
                    PlayMotionAction,
                    goal=PlayMotionGoal(
                        motion_name="close_gripper", skip_planning=False
                    ),
                ),
                transitions={"succeeded": "succeeded", "aborted": "failed"},
            )
            smach.StateMachine.add(
                "FOLD_ARM",
                SimpleActionState(
                    "play_motion",
                    PlayMotionAction,
                    goal=PlayMotionGoal(motion_name="fold_arm", skip_planning=False),
                ),
                transitions={"succeeded": "succeeded", "aborted": "failed"},
            )
