import smach

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
    def __init__(self, node, last_resort, guest_id):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data"],
            output_keys=["guest_data", "person_detections", ""],
        )
        self.add_input_key('guest_data')
        self.add_output_key('guest_data')
        self.add_output_key('person_detections')

        with self:
            conc_face_attribute = smach.Concurrence(
                outcomes=["succeeded", "failed", "failed_attributes", "failed_face"],
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
                input_keys=["guest_data"],
                output_keys=["guest_data"],
            )

            conc_name_drink_face = smach.Concurrence(
                outcomes=[
                    "succeeded",
                    "failed",
                    "failed_vision",
                    "failed_attributes",
                    "failed_face",
                ],
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
                input_keys=["guest_data", "guest_transcription"],
                output_keys=["guest_data"],
            )

            with conc_face_attribute:
                conc_face_attribute.add(
                    "GET_ATTRIBUTES", GetGuestAttributes(node=node, guest_id=guest_id)
                )
                conc_face_attribute.add(
                    "LEARN_FACE", HRILearnFaces(node=node, guest_id=guest_id)
                )

            with conc_name_drink_face:
                conc_name_drink_face.add(
                    "GET_NAME_DRINK",
                    GetNameAndDrink(
                        node=node, guest_id=guest_id, last_resort=last_resort
                    ),
                )
                conc_name_drink_face.add("GET_FACE_ATTRIBUTES", conc_face_attribute)

            self.add(
                "SAY_WAITING_FOR_GUEST",
                Say(node=node, text="I am waiting for a guest."),
                transitions={
                    "succeeded": "WAIT_FOR_GUEST",
                    "aborted": "WAIT_FOR_GUEST",
                    "preempted": "WAIT_FOR_GUEST",
                },
            )
            self.add(
                "WAIT_FOR_GUEST",
                WaitForPersonInArea(node=node, area_polygon_param="door_polygon"),
                transitions={
                    "succeeded": "GET_PERSON_POINT",
                    "failed": "SAY_WAITING_FOR_GUEST",
                },
                remapping={"detections_3d": "person_detections"},
            )
            self.add(
                "GET_PERSON_POINT",
                GetPersonPoint(node=node),
                transitions={
                    "succeeded": "START_EYE_TRACKER",
                    "failed": "SAY_WAITING_FOR_GUEST",
                },
            )
            self.add(
                "START_EYE_TRACKER",
                StartEyeTracker(node=node),
                transitions={
                    "succeeded": "GREET_AND_ASK_GUEST",
                    "aborted": "SAY_WAITING_FOR_GUEST",
                    "preempted": "failed",
                },
            )
            self.add(
                "GREET_AND_ASK_GUEST",
                AskAndListen(
                    node=node,
                    tts_phrase="Please say 'Hi Tiago' for me to begin listening. What is your name and drink?",
                ),
                transitions={
                    "succeeded": "GET_NAME_DRINK_FACE",
                    "failed": "GREET_AND_ASK_GUEST",
                },
                remapping={"transcribed_speech": "guest_transcription"},
            )
            self.add(
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
