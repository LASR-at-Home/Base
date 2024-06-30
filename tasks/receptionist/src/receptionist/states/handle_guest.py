import smach

from lasr_skills import AskAndListen, Say
from receptionist.states import (
    ParseNameAndDrink,
    ParseTranscribedInfo,
    ReceptionistLearnFaces,
    GetGuestAttributes,
)


class HandleGuest(smach.StateMachine):

    class GetNameAndDrink(smach.StateMachine):
        def __init__(self, guest_id: str):
            super().__init__(
                outcomes=["succeeded", "failed"],
                input_keys=["guest_data"],
                output_keys=["guest_data"],
            )

            with self:
                smach.StateMachine.add(
                    f"GET_NAME_AND_DRINK_GUEST_{guest_id}",
                    AskAndListen("What is your name and favourite drink?"),
                    transitions={
                        "succeeded": f"PARSE_NAME_AND_DRINK_GUEST_{guest_id}",
                        "failed": f"PARSE_NAME_AND_DRINK_GUEST_{guest_id}",
                    },
                )

                smach.StateMachine.add(
                    f"PARSE_NAME_AND_DRINK_GUEST_{guest_id}",
                    ParseNameAndDrink(guest_id),
                    transitions={
                        "succeeded": "succeeded",
                        "failed": f"REPEAT_GET_NAME_AND_DRINK_GUEST_{guest_id}",
                        "failed_name": f"REPEAT_GET_NAME_GUEST_{guest_id}",
                        "failed_drink": f"REPEAT_GET_DRINK_GUEST_{guest_id}",
                    },
                    remapping={"guest_transcription": "transcribed_speech"},
                )

                smach.StateMachine.add(
                    f"REPEAT_GET_NAME_AND_DRINK_GUEST_{guest_id}",
                    AskAndListen(
                        "Sorry, I didn't get that, consider raising your voice. What is your name and favourite drink?"
                    ),
                    transitions={
                        "succeeded": f"REPEAT_PARSE_NAME_AND_DRINK_GUEST_{guest_id}",
                        "failed": "SAY_CONTINUE",
                    },
                )

                smach.StateMachine.add(
                    f"REPEAT_PARSE_NAME_AND_DRINK_GUEST_{guest_id}",
                    ParseNameAndDrink(guest_id),
                    transitions={
                        "succeeded": "succeeded",
                        "failed": "SAY_CONTINUE",
                        "failed_name": "SAY_CONTINUE",
                        "failed_drink": "SAY_CONTINUE",
                    },
                    remapping={"guest_transcription": "transcribed_speech"},
                )

                """
                Recovery for only name not recognised
                """

                smach.StateMachine.add(
                    f"REPEAT_GET_NAME_GUEST_{guest_id}",
                    AskAndListen(
                        "Sorry, I didn't get your name. What is your name? Feel free to repeat your name several times."
                    ),
                    transitions={
                        "succeeded": f"REPEAT_PARSE_NAME_GUEST_{guest_id}",
                        "failed": "SAY_CONTINUE",
                    },
                )

                smach.StateMachine.add(
                    f"REPEAT_PARSE_NAME_GUEST_{guest_id}",
                    ParseTranscribedInfo(guest_id, "name"),
                    transitions={
                        "succeeded": "succeeded",
                        "failed": "SAY_CONTINUE",
                    },
                    remapping={"guest_transcription": "transcribed_speech"},
                )

                """
                Recovery for only drink not recognised
                """

                smach.StateMachine.add(
                    f"REPEAT_GET_DRINK_GUEST_{guest_id}",
                    AskAndListen(
                        "Sorry, I didn't get your favourite drink. What is your favourite drink? Feel free to repeat your favourite drink several times."
                    ),
                    transitions={
                        "succeeded": f"REPEAT_PARSE_DRINK_GUEST_{guest_id}",
                        "failed": "SAY_CONTINUE",
                    },
                )

                smach.StateMachine.add(
                    f"REPEAT_PARSE_DRINK_GUEST_{guest_id}",
                    ParseTranscribedInfo(guest_id, "drink"),
                    transitions={
                        "succeeded": "succeeded",
                        "failed": "SAY_CONTINUE",
                    },
                    remapping={"guest_transcription": "transcribed_speech"},
                )

                """
                Recovery if nothing was recognised (twice)
                """
                smach.StateMachine.add(
                    "SAY_CONTINUE",
                    Say(text="Sorry, I didn't get that. I will carry on."),
                    transitions={
                        "succeeded": "succeeded",
                        "aborted": "succeeded",
                        "preempted": "succeeded",
                    },
                )

    def __init__(self, guest_id: str):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data"],
        )

        with self:
            sm_con = smach.Concurrence(
                outcomes=["succeeded", "failed"],
                default_outcome="failed",
                outcome_map={
                    "succeeded": {
                        "GET_NAME_AND_DRINK": "succeeded",
                        "GET_ATTRIBUTES": "succeeded",
                        "LEARN_FACE": "succeeded",
                    }
                },
                input_keys=["guest_data"],
                output_keys=["guest_data"],
            )

            with sm_con:
                smach.Concurrence.add(
                    "GET_NAME_AND_DRINK",
                    self.GetNameAndDrink(guest_id),
                )

                smach.Concurrence.add(
                    "LEARN_FACE",
                    ReceptionistLearnFaces(guest_id),
                )

                smach.Concurrence.add(
                    "GET_ATTRIBUTES",
                    GetGuestAttributes(guest_id),
                )

            smach.StateMachine.add(
                "HANDLE_GUEST", sm_con, transitions={"succeeded": "succeeded"}
            )
