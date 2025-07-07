import smach

<<<<<<< HEAD
from lasr_skills import AskAndListen, Say
=======
from lasr_skills import AskAndListen, Say, StartEyeTracker, StopEyeTracker
>>>>>>> origin/main
from receptionist.states import (
    GetNameAndInterest,
    ReceptionistLearnFaces,
    GetGuestAttributes,
)


<<<<<<< HEAD
=======
class GetPersonPoint(smach.State):
    """State to get the point of interest of the person."""

<<<<<<<< HEAD:tasks/receptionist/src/receptionist/states/handle_guest.py
    class HandleNameAndInterest(smach.StateMachine):
========
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["person_detections"],
            output_keys=["person_point"],
        )

    def execute(self, userdata):
        if not userdata.person_detections:
            return "failed"
        # Assuming the first detection is the point of interest
        userdata.person_point = userdata.person_detections[0].point
        return "succeeded"


>>>>>>> origin/main
class HandleNameInterest(smach.StateMachine):
    def __init__(self, guest_id: str):
        super().__init__(
            outcomes=[
                "succeeded",
                "failed",
            ],
<<<<<<< HEAD
            input_keys=["guest_data"],
=======
            input_keys=["guest_data", "person_detections"],
>>>>>>> origin/main
        )

        with self:
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
                        "GET_NAME_INTEREST": "succeeded",
                        "GET_ATTRIBUTES_AND_LEARN_FACE": "succeeded",
                    },
                    "failed": {
                        "GET_NAME_INTEREST": "failed",
                        "GET_ATTRIBUTES_AND_LEARN_FACE": "failed",
                    },
                    "vision_failed": {
                        "GET_NAME_INTEREST": "succeeded",
                        "GET_ATTRIBUTES_AND_LEARN_FACE": "failed",
                    },
                    "get_attributes_failed": {
                        "GET_NAME_INTEREST": "succeeded",
                        "GET_ATTRIBUTES_AND_LEARN_FACE": "get_attributes_failed",
                    },
                    "learn_face_failed": {
                        "GET_NAME_INTEREST": "succeeded",
                        "GET_ATTRIBUTES_AND_LEARN_FACE": "learn_face_failed",
                    },
                },
                input_keys=["guest_data"],
                output_keys=["guest_data"],
            )

            with sm_con:
                smach.Concurrence.add(
                    "GET_NAME_INTEREST",
                    self.NameInterestFlow(guest_id),
                )

                smach.Concurrence.add(
                    "GET_ATTRIBUTES_AND_LEARN_FACE",
                    self.GetAttributesAndLearnFace(guest_id),
                )

            smach.StateMachine.add(
<<<<<<< HEAD
=======
                "GET_PERSON_POINT",
                GetPersonPoint(),
                transitions={
                    "succeeded": "START_EYE_TRACKER",
                    "failed": "HANDLE_NAME_INTEREST",
                },
                remapping={
                    "person_detections": "person_detections",
                    "person_point": "person_point",
                },
            )

            smach.StateMachine.add(
                "START_EYE_TRACKER",
                StartEyeTracker(),
                transitions={
                    "succeeded": "HANDLE_NAME_INTEREST",
                    "failed": "HANDLE_NAME_INTEREST",
                },
            )

            smach.StateMachine.add(
>>>>>>> origin/main
                "HANDLE_NAME_INTEREST",
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
                Say(text="Please look into my eyes."),
                transitions={
                    "succeeded": "GET_ATTRIBUTES_AND_LEARN_FACE",
                    "aborted": "GET_ATTRIBUTES_AND_LEARN_FACE",
                    "preempted": "GET_ATTRIBUTES_AND_LEARN_FACE",
                },
            )

            smach.StateMachine.add(
                "GET_ATTRIBUTES_AND_LEARN_FACE",
                self.GetAttributesAndLearnFace(guest_id),
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

    class NameInterestFlow(smach.StateMachine):
<<<<<<< HEAD
=======
>>>>>>>> origin/main:tasks/receptionist/src/receptionist/states/handle_name_interest.py
>>>>>>> origin/main
        def __init__(self, guest_id: str):
            super().__init__(
                outcomes=["succeeded", "failed"],
                input_keys=["guest_data"],
                output_keys=["guest_data"],
            )

            with self:
                smach.StateMachine.add(
<<<<<<< HEAD
                    f"GET_NAME_INTEREST_{guest_id}",
=======
<<<<<<<< HEAD:tasks/receptionist/src/receptionist/states/handle_guest.py
                    f"GET_NAME_AND_INTEREST_GUEST_{guest_id}",
========
                    f"GET_NAME_INTEREST_{guest_id}",
>>>>>>>> origin/main:tasks/receptionist/src/receptionist/states/handle_name_interest.py
>>>>>>> origin/main
                    AskAndListen(
                        "Please say 'Hi Tiago' for me to begin listening. What is your name and interest?"
                    ),
                    transitions={
<<<<<<< HEAD
                        "succeeded": f"PARSE_NAME_INTEREST_{guest_id}",
                        "failed": f"PARSE_NAME_INTEREST_{guest_id}",
=======
<<<<<<<< HEAD:tasks/receptionist/src/receptionist/states/handle_guest.py
                        "succeeded": f"PARSE_NAME_AND_INTEREST_GUEST_{guest_id}",
                        "failed": f"PARSE_NAME_AND_INTEREST_GUEST_{guest_id}",
========
                        "succeeded": f"PARSE_NAME_INTEREST_{guest_id}",
                        "failed": f"PARSE_NAME_INTEREST_{guest_id}",
>>>>>>>> origin/main:tasks/receptionist/src/receptionist/states/handle_name_interest.py
>>>>>>> origin/main
                    },
                )

                smach.StateMachine.add(
<<<<<<< HEAD
                    f"PARSE_NAME_INTEREST_{guest_id}",
                    GetNameAndInterest(guest_id, False),
                    transitions={
                        "succeeded": "succeeded",
                        "failed": f"REPEAT_GET_NAME_INTEREST_{guest_id}",
=======
<<<<<<<< HEAD:tasks/receptionist/src/receptionist/states/handle_guest.py
                    f"PARSE_NAME_AND_INTEREST_GUEST_{guest_id}",
                    GetNameAndInterest(guest_id, False),
                    transitions={
                        "succeeded": "succeeded",
                        "failed": f"REPEAT_GET_NAME_AND_INTEREST_GUEST_{guest_id}",
                        "failed_name": f"REPEAT_GET_NAME_GUEST_{guest_id}",
                        "failed_interest": f"REPEAT_GET_INTEREST_GUEST_{guest_id}",
========
                    f"PARSE_NAME_INTEREST_{guest_id}",
                    GetNameAndInterest(guest_id, False),
                    transitions={
                        "succeeded": f"succeeded",
                        "failed": f"REPEAT_GET_NAME_INTEREST_{guest_id}",
>>>>>>>> origin/main:tasks/receptionist/src/receptionist/states/handle_name_interest.py
>>>>>>> origin/main
                    },
                    remapping={"guest_transcription": "transcribed_speech"},
                )

                smach.StateMachine.add(
<<<<<<< HEAD
                    f"REPEAT_GET_NAME_INTEREST_{guest_id}",
=======
<<<<<<<< HEAD:tasks/receptionist/src/receptionist/states/handle_guest.py
                    f"REPEAT_GET_NAME_AND_INTEREST_GUEST_{guest_id}",
========
                    f"REPEAT_GET_NAME_INTEREST_{guest_id}",
>>>>>>>> origin/main:tasks/receptionist/src/receptionist/states/handle_name_interest.py
>>>>>>> origin/main
                    AskAndListen(
                        "Please speak louder. What is your name and interest?",
                    ),
                    transitions={
<<<<<<< HEAD
                        "succeeded": f"REPEAT_PARSE_NAME_INTEREST_{guest_id}",
=======
<<<<<<<< HEAD:tasks/receptionist/src/receptionist/states/handle_guest.py
                        "succeeded": f"REPEAT_PARSE_NAME_AND_INTEREST_GUEST_{guest_id}",
========
                        "succeeded": f"REPEAT_PARSE_NAME_INTEREST_{guest_id}",
>>>>>>>> origin/main:tasks/receptionist/src/receptionist/states/handle_name_interest.py
>>>>>>> origin/main
                        "failed": "succeeded",
                    },
                )

                smach.StateMachine.add(
<<<<<<< HEAD
                    f"REPEAT_PARSE_NAME_INTEREST_{guest_id}",
=======
<<<<<<<< HEAD:tasks/receptionist/src/receptionist/states/handle_guest.py
                    f"REPEAT_PARSE_NAME_AND_INTEREST_GUEST_{guest_id}",
>>>>>>> origin/main
                    GetNameAndInterest(guest_id, True),
                    transitions={
                        "succeeded": "succeeded",
                        "failed": "succeeded",
<<<<<<< HEAD
=======
                        "failed_name": "succeeded",
                        "failed_interest": "succeeded",
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
                    GetNameOrInterest(guest_id, True, "name"),
                    transitions={
                        "succeeded": "succeeded",
                        "failed": "succeeded",
>>>>>>> origin/main
                    },
                    remapping={"guest_transcription": "transcribed_speech"},
                )

<<<<<<< HEAD
=======
                """
                Recovery for only interest not recognised
                """

                smach.StateMachine.add(
                    f"REPEAT_GET_INTEREST_GUEST_{guest_id}",
                    AskAndListen("Please speak louder. What is your interest?"),
                    transitions={
                        "succeeded": f"REPEAT_PARSE_INTEREST_GUEST_{guest_id}",
                        "failed": "succeeded",
                    },
                )

                smach.StateMachine.add(
                    f"REPEAT_PARSE_INTEREST_GUEST_{guest_id}",
                    GetNameOrInterest(guest_id, True, "interest"),
                    transitions={
                        "succeeded": "succeeded",
                        "failed": "succeeded",
========
                    f"REPEAT_PARSE_NAME_INTEREST_{guest_id}",
                    GetNameAndInterest(guest_id, True),
                    transitions={
                        "succeeded": f"succeeded",
                        "failed": f"succeeded",
>>>>>>>> origin/main:tasks/receptionist/src/receptionist/states/handle_name_interest.py
                    },
                    remapping={"guest_transcription": "transcribed_speech"},
                )

    # class HandleNameAndDrink(smach.StateMachine):
    #     def __init__(self, guest_id: str):
    #         super().__init__(
    #             outcomes=["succeeded", "failed"],
    #             input_keys=["guest_data"],
    #             output_keys=["guest_data"],
    #         )

    #         with self:
    #             smach.StateMachine.add(
    #                 f"GET_NAME_AND_DRINK_GUEST_{guest_id}",
    #                 AskAndListen(
    #                     "Please say 'Hi Tiago' for me to begin listening. What is your name and favourite drink?"
    #                 ),
    #                 transitions={
    #                     "succeeded": f"PARSE_NAME_AND_DRINK_GUEST_{guest_id}",
    #                     "failed": f"PARSE_NAME_AND_DRINK_GUEST_{guest_id}",
    #                 },
    #             )

    #             smach.StateMachine.add(
    #                 f"PARSE_NAME_AND_DRINK_GUEST_{guest_id}",
    #                 GetNameAndDrink(guest_id, False),
    #                 transitions={
    #                     "succeeded": "succeeded",
    #                     "failed": f"REPEAT_GET_NAME_AND_DRINK_GUEST_{guest_id}",
    #                     "failed_name": f"REPEAT_GET_NAME_GUEST_{guest_id}",
    #                     "failed_drink": f"REPEAT_GET_DRINK_GUEST_{guest_id}",
    #                 },
    #                 remapping={"guest_transcription": "transcribed_speech"},
    #             )

    #             smach.StateMachine.add(
    #                 f"REPEAT_GET_NAME_AND_DRINK_GUEST_{guest_id}",
    #                 AskAndListen(
    #                     "Please speak louder. What is your name and favourite drink?",
    #                 ),
    #                 transitions={
    #                     "succeeded": f"REPEAT_PARSE_NAME_AND_DRINK_GUEST_{guest_id}",
    #                     "failed": "succeeded",
    #                 },
    #             )

    #             smach.StateMachine.add(
    #                 f"REPEAT_PARSE_NAME_AND_DRINK_GUEST_{guest_id}",
    #                 GetNameAndDrink(guest_id, True),
    #                 transitions={
    #                     "succeeded": "succeeded",
    #                     "failed": "succeeded",
    #                     "failed_name": "succeeded",
    #                     "failed_drink": "succeeded",
    #                 },
    #                 remapping={"guest_transcription": "transcribed_speech"},
    #             )

    #             """
    #             Recovery for only name not recognised
    #             """

    #             smach.StateMachine.add(
    #                 f"REPEAT_GET_NAME_GUEST_{guest_id}",
    #                 AskAndListen("Plase speak louader. What is your name?"),
    #                 transitions={
    #                     "succeeded": f"REPEAT_PARSE_NAME_GUEST_{guest_id}",
    #                     "failed": "succeeded",
    #                 },
    #             )

    #             smach.StateMachine.add(
    #                 f"REPEAT_PARSE_NAME_GUEST_{guest_id}",
    #                 GetNameOrDrink(guest_id, True, "name"),
    #                 transitions={
    #                     "succeeded": "succeeded",
    #                     "failed": "succeeded",
    #                 },
    #                 remapping={"guest_transcription": "transcribed_speech"},
    #             )

    #             """
    #             Recovery for only drink not recognised
    #             """

    #             smach.StateMachine.add(
    #                 f"REPEAT_GET_DRINK_GUEST_{guest_id}",
    #                 AskAndListen("Please speak louder. What is your favourite drink?"),
    #                 transitions={
    #                     "succeeded": f"REPEAT_PARSE_DRINK_GUEST_{guest_id}",
    #                     "failed": "succeeded",
    #                 },
    #             )

    #             smach.StateMachine.add(
    #                 f"REPEAT_PARSE_DRINK_GUEST_{guest_id}",
    #                 GetNameOrDrink(guest_id, True, "drink"),
    #                 transitions={
    #                     "succeeded": "succeeded",
    #                     "failed": "succeeded",
    #                 },
    #                 remapping={"guest_transcription": "transcribed_speech"},
    #             )

>>>>>>> origin/main
    class GetAttributesAndLearnFace(smach.StateMachine):

        def __init__(self, guest_id: str):

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

                    smach.Concurrence.add(
                        "LEARN_FACE",
                        ReceptionistLearnFaces(guest_id),
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
<<<<<<< HEAD
=======
<<<<<<<< HEAD:tasks/receptionist/src/receptionist/states/handle_guest.py

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
                        "GET_NAME_AND_INTEREST": "succeeded",
                        "GET_ATTRIBUTES_AND_LEARN_FACE": "succeeded",
                    },
                    "failed": {
                        "GET_NAME_AND_INTEREST": "failed",
                        "GET_ATTRIBUTES_AND_LEARN_FACE": "failed",
                    },
                    "vision_failed": {
                        "GET_NAME_AND_INTEREST": "succeeded",
                        "GET_ATTRIBUTES_AND_LEARN_FACE": "failed",
                    },
                    "get_attributes_failed": {
                        "GET_NAME_AND_INTEREST": "succeeded",
                        "GET_ATTRIBUTES_AND_LEARN_FACE": "get_attributes_failed",
                    },
                    "learn_face_failed": {
                        "GET_NAME_AND_INTEREST": "succeeded",
                        "GET_ATTRIBUTES_AND_LEARN_FACE": "learn_face_failed",
                    },
                },
                input_keys=["guest_data"],
                output_keys=["guest_data"],
            )

            with sm_con:
                smach.Concurrence.add(
                    "GET_NAME_AND_INTEREST",
                    self.HandleNameAndInterest(guest_id),
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
========
>>>>>>>> origin/main:tasks/receptionist/src/receptionist/states/handle_name_interest.py
>>>>>>> origin/main
