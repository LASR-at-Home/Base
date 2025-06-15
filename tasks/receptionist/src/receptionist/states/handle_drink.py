import smach

from lasr_skills import AskAndListen
from receptionist.states import (
    GetDrink,
)


class HandleDrink(smach.StateMachine):
    def __init__(self, guest_id: str):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data"],
        )

        with self:
            smach.StateMachine.add(
                "GET_DRINK",
                self.DrinkFlow(guest_id),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )

    class DrinkFlow(smach.StateMachine):
        def __init__(self, guest_id: str):
            super().__init__(
                outcomes=["succeeded", "failed"],
                input_keys=["guest_data"],
                output_keys=["guest_data"],
            )

            with self:
                smach.StateMachine.add(
                    f"GET_DRINK_GUEST_{guest_id}",
                    AskAndListen(
                        "Please say 'Hi Tiago' for me to begin listening. What is your favourite drink?"
                    ),
                    transitions={
                        "succeeded": f"PARSE_DRINK_GUEST_{guest_id}",
                        "failed": f"PARSE_DRINK_GUEST_{guest_id}",
                    },
                )

                smach.StateMachine.add(
                    f"PARSE_DRINK_GUEST_{guest_id}",
                    GetDrink(guest_id, False),
                    transitions={
                        "succeeded": "succeeded",
                        "failed": f"REPEAT_GET_DRINK_GUEST_{guest_id}",
                    },
                    remapping={"guest_transcription": "transcribed_speech"},
                )

                smach.StateMachine.add(
                    f"REPEAT_GET_DRINK_GUEST_{guest_id}",
                    AskAndListen(
                        "Please speak louder. What is your favourite drink?",
                    ),
                    transitions={
                        "succeeded": f"REPEAT_PARSE_DRINK_GUEST_{guest_id}",
                        "failed": "succeeded",
                    },
                )

                smach.StateMachine.add(
                    f"REPEAT_PARSE_DRINK_GUEST_{guest_id}",
                    GetDrink(guest_id, True),
                    transitions={
                        "succeeded": "succeeded",
                        "failed": "succeeded",
                    },
                    remapping={"guest_transcription": "transcribed_speech"},
                )
