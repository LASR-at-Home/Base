import smach

from lasr_skills import AskAndListen, Say
from storing_groceries.states import (
    GetInterest,
)


class HandleInterest(smach.StateMachine):

    class InterestFlow(smach.StateMachine):
        def __init__(self, guest_id: str):
            super().__init__(
                outcomes=["succeeded", "failed"],
                input_keys=["guest_data"],
                output_keys=["guest_data"],
            )

            with self:
                smach.StateMachine.add(
                    f"GET_INTEREST_GUEST_{guest_id}",
                    AskAndListen(
                        "Please say 'Hi Tiago' for me to begin listening. What is your interest?"
                    ),
                    transitions={
                        "succeeded": f"PARSE_INTEREST_GUEST_{guest_id}",
                        "failed": f"PARSE_INTEREST_GUEST_{guest_id}",
                    },
                )

                smach.StateMachine.add(
                    f"PARSE_INTEREST_GUEST_{guest_id}",
                    GetInterest(guest_id, False),
                    transitions={
                        "succeeded": "succeeded",
                        "failed": f"REPEAT_GET_INTEREST_GUEST_{guest_id}",
                    },
                    remapping={"guest_transcription": "transcribed_speech"},
                )

                smach.StateMachine.add(
                    f"REPEAT_GET_INTEREST_GUEST_{guest_id}",
                    AskAndListen(
                        "Please speak louder. What is your interest?",
                    ),
                    transitions={
                        "succeeded": f"REPEAT_PARSE_INTEREST_GUEST_{guest_id}",
                        "failed": "succeeded",
                    },
                )

                smach.StateMachine.add(
                    f"REPEAT_PARSE_INTEREST_GUEST_{guest_id}",
                    GetInterest(guest_id, True),
                    transitions={
                        "succeeded": "succeeded",
                        "failed": "succeeded",
                    },
                    remapping={"guest_transcription": "transcribed_speech"},
                )

    def __init__(self, guest_id: str):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data"],
        )

        with self:
            smach.StateMachine.add(
                "GET_INTEREST",
                self.InterestFlow(guest_id),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )