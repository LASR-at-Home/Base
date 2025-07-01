import smach

from lasr_skills import AskAndListen, Say, StartEyeTracker, StopEyeTracker
from receptionist.states import (
    GetNameAndInterest,
    ReceptionistLearnFaces,
    GetGuestAttributes,
)


class GetPersonPoint(smach.State):
    """State to get the point of interest of the person."""

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


class HandleNameInterest(smach.StateMachine):
    def __init__(self, guest_id: str):
        super().__init__(
            outcomes=[
                "succeeded",
                "failed",
            ],
            input_keys=["guest_data", "person_detections"],
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
        def __init__(self, guest_id: str):
            super().__init__(
                outcomes=["succeeded", "failed"],
                input_keys=["guest_data"],
                output_keys=["guest_data"],
            )

            with self:
                smach.StateMachine.add(
                    f"GET_NAME_INTEREST_{guest_id}",
                    AskAndListen(
                        "Please say 'Hi Tiago' for me to begin listening. What is your name and interest?"
                    ),
                    transitions={
                        "succeeded": f"PARSE_NAME_INTEREST_{guest_id}",
                        "failed": f"PARSE_NAME_INTEREST_{guest_id}",
                    },
                )

                smach.StateMachine.add(
                    f"PARSE_NAME_INTEREST_{guest_id}",
                    GetNameAndInterest(guest_id, False),
                    transitions={
                        "succeeded": f"succeeded",
                        "failed": f"REPEAT_GET_NAME_INTEREST_{guest_id}",
                    },
                    remapping={"guest_transcription": "transcribed_speech"},
                )

                smach.StateMachine.add(
                    f"REPEAT_GET_NAME_INTEREST_{guest_id}",
                    AskAndListen(
                        "Please speak louder. What is your name and interest?",
                    ),
                    transitions={
                        "succeeded": f"REPEAT_PARSE_NAME_INTEREST_{guest_id}",
                        "failed": "succeeded",
                    },
                )

                smach.StateMachine.add(
                    f"REPEAT_PARSE_NAME_INTEREST_{guest_id}",
                    GetNameAndInterest(guest_id, True),
                    transitions={
                        "succeeded": f"succeeded",
                        "failed": f"succeeded",
                    },
                    remapping={"guest_transcription": "transcribed_speech"},
                )

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
