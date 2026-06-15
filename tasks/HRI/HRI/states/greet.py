import yasmin

from lasr_skills import Say, StartEyeTracker, WaitForPersonInArea, AskAndListen
from HRI.states import (
    GetNameAndDrink,
    GetGuestAttributes,
    HRILearnFaces,
    GetPersonPoint,
)

"""
Robot is already at door, since:
    SM1 detects door ---(door opening)---> robot goes through door to waiting area ---(waits a few seconds)---> robot goes back to door area
    
So Robot is at door and it:
    Faces person -> Start Eye tracker -> Greet guest -> Listen -> recognise name and drink -> listen loop
"""


class LookAndGreetGuest(yasmin.StateMachine):
    def __init__(self, last_resort, guest_id):
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)
        self.add_input_key("guest_data")
        self.add_output_key("guest_data")
        self.add_output_key("person_detections")

        conc_face_attribute = yasmin.Concurrence(
            states={
                "GET_ATTRIBUTES": GetGuestAttributes(guest_id=guest_id),
                "LEARN_FACE": HRILearnFaces(guest_id=guest_id),
            },
            default_outcome="failed",
            outcome_map={
                "succeeded": {
                    "GET_ATTRIBUTES": "succeeded",
                    "LEARN_FACE": "succeeded",
                },
                "failed": {"GET_ATTRIBUTES": "failed", "LEARN_FACE": "failed"},
                "failed_attributes": {
                    "GET_ATTRIBUTES": "failed",
                    "LEARN_FACE": "succeeded",
                },
                "failed_face": {
                    "GET_ATTRIBUTES": "succeeded",
                    "LEARN_FACE": "failed",
                },
            },
        )

        conc_face_attribute.add_input_key("guest_data")
        conc_face_attribute.add_output_key("guest_data")

        conc_name_drink_face = yasmin.Concurrence(
            states={
                "GET_NAME_DRINK": GetNameAndDrink(
                    guest_id=guest_id, last_resort=last_resort
                ),
                "GET_FACE_ATTRIBUTES": conc_face_attribute,
            },
            default_outcome="failed",
            outcome_map={
                "succeeded": {
                    "GET_NAME_DRINK": "succeeded",
                    "GET_FACE_ATTRIBUTES": "succeeded",
                },
                "failed": {
                    "GET_NAME_DRINK": "failed",
                    "GET_FACE_ATTRIBUTES": "failed",
                },
                "failed_vision": {
                    "GET_NAME_DRINK": "succeeded",
                    "GET_FACE_ATTRIBUTES": "failed",
                },
                "failed_attributes": {
                    "GET_NAME_DRINK": "succeeded",
                    "GET_FACE_ATTRIBUTES": "failed_attributes",
                },
                "failed_face": {
                    "GET_NAME_DRINK": "succeeded",
                    "GET_FACE_ATTRIBUTES": "failed_face",
                },
            },
        )

        conc_name_drink_face.add_input_key("guest_data")
        conc_name_drink_face.add_input_key("guest_data")
        conc_name_drink_face.add_output_key("guest_data")

        self.add_state(
            "SAY_WAITING_FOR_GUEST",
            Say(text="I am waiting for a guest."),
            transitions={
                "succeeded": "WAIT_FOR_GUEST",
                "aborted": "WAIT_FOR_GUEST",
                "canceled": "WAIT_FOR_GUEST",
            },
        )
        self.add_state(
            "WAIT_FOR_GUEST",
            WaitForPersonInArea(polygon_param="door_polygon"),
            transitions={
                "succeeded": "GET_PERSON_POINT",
                "failed": "SAY_WAITING_FOR_GUEST",
            },
            remappings={"detections_3d": "person_detections"},
        )
        self.add_state(
            "GET_PERSON_POINT",
            GetPersonPoint(),
            transitions={
                "succeeded": "START_EYE_TRACKER",
                "failed": "SAY_WAITING_FOR_GUEST",
            },
        )
        self.add_state(
            "START_EYE_TRACKER",
            StartEyeTracker(),
            transitions={
                "succeeded": "GREET_AND_ASK_GUEST",
                "aborted": "SAY_WAITING_FOR_GUEST",
                "canceled": "failed",
                "timeout": "GREET_AND_ASK_GUEST",
                "timeout": "GREET_AND_ASK_GUEST",
            },
        )
        self.add_state(
            "GREET_AND_ASK_GUEST",
            AskAndListen(
                tts_phrase="Please say 'Hi Tiago' for me to begin listening. What is your name and drink?",
            ),
            transitions={
                "succeeded": "GET_NAME_DRINK_FACE",
                "failed": "GREET_AND_ASK_GUEST",
            },
            remappings={"transcribed_speech": "guest_transcription"},
        )
        self.add_state(
            "GET_NAME_DRINK_FACE",
            conc_name_drink_face,
            transitions={
                "succeeded": "succeeded",
                "failed": "failed",
                "failed_vision": "failed",
                "failed_face": "failed",
                "failed_attributes": "failed",
            },
        )
