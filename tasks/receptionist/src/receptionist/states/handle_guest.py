import smach

from lasr_skills import AskAndListen, Say, AdjustCamera
from receptionist.states import (
    GetNameAndDrink,
    GetNameOrDrink,
    ReceptionistLearnFaces,
    GetGuestAttributes,
)


class HandleGuest(smach.StateMachine):

    class HandleNameAndDrink(smach.StateMachine):
        def __init__(self, guest_id: str):
            super().__init__(
                outcomes=["succeeded", "failed"],
                input_keys=["guest_data"],
                output_keys=["guest_data"],
            )

            with self:
                smach.StateMachine.add(
                    f"GET_NAME_AND_DRINK_GUEST_{guest_id}",
                    AskAndListen(
                        "Please say 'Hi Tiago' for me to begin listening. What is your name and favourite drink?"
                    ),
                    transitions={
                        "succeeded": f"PARSE_NAME_AND_DRINK_GUEST_{guest_id}",
                        "failed": f"PARSE_NAME_AND_DRINK_GUEST_{guest_id}",
                    },
                )

                smach.StateMachine.add(
                    f"PARSE_NAME_AND_DRINK_GUEST_{guest_id}",
                    GetNameAndDrink(guest_id, False),
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
                        "Please speak louder. What is your name and favourite drink?",
                    ),
                    transitions={
                        "succeeded": f"REPEAT_PARSE_NAME_AND_DRINK_GUEST_{guest_id}",
                        "failed": "succeeded",
                    },
                )

                smach.StateMachine.add(
                    f"REPEAT_PARSE_NAME_AND_DRINK_GUEST_{guest_id}",
                    GetNameAndDrink(guest_id, True),
                    transitions={
                        "succeeded": "succeeded",
                        "failed": "succeeded",
                        "failed_name": "succeeded",
                        "failed_drink": "succeeded",
                    },
                    remapping={"guest_transcription": "transcribed_speech"},
                )

                """
                Recovery for only name not recognised
                """

                smach.StateMachine.add(
                    f"REPEAT_GET_NAME_GUEST_{guest_id}",
                    AskAndListen("Plase speak louader. What is your name?"),
                    transitions={
                        "succeeded": f"REPEAT_PARSE_NAME_GUEST_{guest_id}",
                        "failed": "succeeded",
                    },
                )

                smach.StateMachine.add(
                    f"REPEAT_PARSE_NAME_GUEST_{guest_id}",
                    GetNameOrDrink(guest_id, True, "name"),
                    transitions={
                        "succeeded": "succeeded",
                        "failed": "succeeded",
                    },
                    remapping={"guest_transcription": "transcribed_speech"},
                )

                """
                Recovery for only drink not recognised
                """

                smach.StateMachine.add(
                    f"REPEAT_GET_DRINK_GUEST_{guest_id}",
                    AskAndListen("Please speak louder. What is your favourite drink?"),
                    transitions={
                        "succeeded": f"REPEAT_PARSE_DRINK_GUEST_{guest_id}",
                        "failed": "succeeded",
                    },
                )

                smach.StateMachine.add(
                    f"REPEAT_PARSE_DRINK_GUEST_{guest_id}",
                    GetNameOrDrink(guest_id, True, "drink"),
                    transitions={
                        "succeeded": "succeeded",
                        "failed": "succeeded",
                    },
                    remapping={"guest_transcription": "transcribed_speech"},
                )

    class GetAttributesAndLearnFace(smach.StateMachine):

        def __init__(self, guest_id: str, learn_face: bool):

            super().__init__(
                outcomes=[
                    "succeeded",
                    "failed",
                    "get_attributes_failed",
                    "learn_face_failed",
                ],
                input_keys=["guest_data"],
            )

            with self:

                sm_con = smach.Concurrence(
                    outcomes=[
                        "succeeded",
                        "failed",
                        "get_attributes_failed",
                        "learn_face_failed",
                    ],
                    default_outcome="failed",
                    outcome_map={
                        "succeeded": {
                            "GET_ATTRIBUTES": "succeeded",
                            "LEARN_FACE": "succeeded",
                        },
                        "failed": {
                            "GET_ATTRIBUTES": "failed",
                            "LEARN_FACE": "failed",
                        },
                        "get_attributes_failed": {
                            "GET_ATTRIBUTES": "failed",
                            "LEARN_FACE": "succeeded",
                        },
                        "learn_face_failed": {
                            "GET_ATTRIBUTES": "succeeded",
                            "LEARN_FACE": "failed",
                        },
                    },
                    input_keys=["guest_data"],
                    output_keys=["guest_data"],
                )

                with sm_con:
                    smach.Concurrence.add(
                        "GET_ATTRIBUTES",
                        GetGuestAttributes(guest_id),
                    )

                    if learn_face:
                        smach.Concurrence.add(
                            "LEARN_FACE",
                            ReceptionistLearnFaces(guest_id),
                        )
                    else:
                        smach.Concurrence.add(
                            "LEARN_FACE",
                            smach.CBState(
                                lambda ud: "succeeded", outcomes=["succeeded", "failed"]
                            ),
                        )

                smach.StateMachine.add(
                    "GET_ATTRIBUTES_AND_LEARN_FACE",
                    sm_con,
                    transitions={
                        "succeeded": "succeeded",
                        "failed": "failed",
                        "get_attributes_failed": "get_attributes_failed",
                        "learn_face_failed": "learn_face_failed",
                    },
                )

    def __init__(self, guest_id: str, learn_face: bool):
        super().__init__(
            outcomes=[
                "succeeded",
                "failed",
            ],
            input_keys=["guest_data"],
        )

        with self:
            smach.StateMachine.add(
                "AdjustCamera",
                AdjustCamera(
                    max_attempts=5,
                    debug=False,
                    init_state="u1m",
                ),
                transitions={
                    "finished": "HANDLE_GUEST",
                    "failed": "HANDLE_GUEST",
                    "truncated": "HANDLE_GUEST",
                },
            )

            sm_con = smach.Concurrence(
                outcomes=[
                    "succeeded",
                    "failed",
                    "vision_failed",
                    "get_attributes_failed",
                    "learn_face_failed",
                ],
                default_outcome="failed",
                outcome_map={
                    "succeeded": {
                        "GET_NAME_AND_DRINK": "succeeded",
                        "GET_ATTRIBUTES_AND_LEARN_FACE": "succeeded",
                    },
                    "failed": {
                        "GET_NAME_AND_DRINK": "failed",
                        "GET_ATTRIBUTES_AND_LEARN_FACE": "failed",
                    },
                    "vision_failed": {
                        "GET_NAME_AND_DRINK": "succeeded",
                        "GET_ATTRIBUTES_AND_LEARN_FACE": "failed",
                    },
                    "get_attributes_failed": {
                        "GET_NAME_AND_DRINK": "succeeded",
                        "GET_ATTRIBUTES_AND_LEARN_FACE": "get_attributes_failed",
                    },
                    "learn_face_failed": {
                        "GET_NAME_AND_DRINK": "succeeded",
                        "GET_ATTRIBUTES_AND_LEARN_FACE": "learn_face_failed",
                    },
                },
                input_keys=["guest_data"],
                output_keys=["guest_data"],
            )

            with sm_con:
                smach.Concurrence.add(
                    "GET_NAME_AND_DRINK",
                    self.HandleNameAndDrink(guest_id),
                )

                smach.Concurrence.add(
                    "GET_ATTRIBUTES_AND_LEARN_FACE",
                    self.GetAttributesAndLearnFace(guest_id, learn_face),
                )

            smach.StateMachine.add(
                "HANDLE_GUEST",
                sm_con,
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                    "vision_failed": "SAY_VISION_FAILED",
                    "get_attributes_failed": "SAY_ATTRIBUTES_FAILED",
                    "learn_face_failed": "SAY_LEARN_FACE_FAILED",
                },
            )

            smach.StateMachine.add(
                "SAY_VISION_FAILED",
                Say(text="Plase look into my eyes."),
                transitions={
                    "succeeded": "GET_ATTRIBUTES_AND_LEARN_FACE",
                    "aborted": "GET_ATTRIBUTES_AND_LEARN_FACE",
                    "preempted": "GET_ATTRIBUTES_AND_LEARN_FACE",
                },
            )

            smach.StateMachine.add(
                "GET_ATTRIBUTES_AND_LEARN_FACE",
                self.GetAttributesAndLearnFace(guest_id, learn_face),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                    "get_attributes_failed": "failed",
                    "learn_face_failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_ATTRIBUTES_FAILED",
                Say(text="Please look into my eyes."),
                transitions={
                    "succeeded": "GET_ATTRIBUTES",
                    "aborted": "GET_ATTRIBUTES",
                    "preempted": "GET_ATTRIBUTES",
                },
            )

            smach.StateMachine.add(
                "GET_ATTRIBUTES",
                GetGuestAttributes(guest_id),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_LEARN_FACE_FAILED",
                Say(text="Please look into my eyes."),
                transitions={
                    "succeeded": "LEARN_FACE",
                    "aborted": "LEARN_FACE",
                    "preempted": "LEARN_FACE",
                },
            )

            smach.StateMachine.add(
                "LEARN_FACE",
                ReceptionistLearnFaces(guest_id),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )
