import yasmin

from lasr_skills import (
    Say,
    StartEyeTracker,
    WaitForPersonInArea,
    AskAndListen,
    ReceiveObject,
    StopEyeTracker,
    Wait,
    SafeGoToLocation
)
from HRI.states import (
    GetNameAndDrink,
    GetGuestAttributes,
    HRILearnFaces,
    GetPersonPoint,
)


class LookAndGreetGuest(yasmin.StateMachine):
    def __init__(self, guest_id):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("guest_data")
        self.add_output_key("guest_data")
        self.add_output_key("person_detections")

        look_and_greet = yasmin.Concurrence(
            states={
                "GREET_ONLY": GreetGuest(last_resort=False, guest_id=guest_id),
                "EYE_TRACKER": StartEyeTracker(),
            },
            default_outcome="failed",
            outcome_map={
                "succeeded": {
                    "GREET_ONLY": "succeeded",
                    "EYE_TRACKER": "canceled",
                }
            },
        )

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
                "succeeded": "LOOK_AND_GREET",
                "failed": "SAY_WAITING_FOR_GUEST",
            },
        )

        self.add_state(
            "LOOK_AND_GREET",
            look_and_greet,
            transitions={"succeeded": "succeeded", "failed": "failed"},
        )


class GreetGuest(yasmin.StateMachine):
    def __init__(self, last_resort, guest_id):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("guest_data")
        self.add_output_key("guest_data")
        self.add_output_key("person_detections")

        attribute = yasmin.CbState(
            outcomes=["succeeded", "failed"], callback=self.get_guest1_attributes
        )

        attribute.add_input_key("guest_data")
        attribute.add_output_key("text")

        conc_face_attribute = yasmin.Concurrence(
            states={
                "GET_ATTRIBUTES": GetGuestAttributes(guest_id=guest_id),
                "LEARN_FACE": HRILearnFaces(guest_id=guest_id, dataset_size=10),
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
                "failed_speech": {
                    "GET_NAME_DRINK": "failed",
                    "GET_FACE_ATTRIBUTES": "succeeded",
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
            "GREET_AND_ASK_GUEST",
            AskAndListen(
                tts_phrase="Please say 'Hi Tiago' for me to begin listening. What is your name and drink?",
            ),
            transitions={
                "succeeded": "SAY_WAIT",
                "failed": "failed",
            },
            remappings={"transcribed_speech": "guest_transcription"},
        )

        transition = "GET_ATTRIBUTE_STR" if guest_id == "guest2" else "SAY_WELCOME"

        self.add_state(
            "SAY_WAIT",
            Say(text='Give me some time to learn your face and attributes. Please wait here.'),
            transitions={
                'succeeded': 'GET_NAME_DRINK_FACE',
                'aborted': 'failed',
                'canceled': 'failed'
            }
        )

        self.add_state(
            "GET_NAME_DRINK_FACE",
            conc_name_drink_face,
            transitions={
                "succeeded": transition,
                "failed": "failed",
                "failed_vision": "failed",
                "failed_speech": "REPEAT_ASK_GUEST",
                "failed_face": "failed",
                "failed_attributes": "failed",
            },
        )

        self.add_state(
            "REPEAT_ASK_GUEST",
            AskAndListen(
                tts_phrase="I am sorry, I did not understand. Please say 'Hi Tiago' for me to begin listening. What is your name and drink?",
            ),
            transitions={
                "succeeded": "GET_NAME_DRINK",
                "failed": "SAY_WELCOME",
            },
            remappings={"transcribed_speech": "guest_transcription"},
        )

        self.add_state(
            "GET_NAME_DRINK",
            GetNameAndDrink(guest_id=guest_id, last_resort=last_resort),
            transitions={
                "succeeded": "SAY_WELCOME",
                "failed": "SAY_WELCOME",
            },
        )

        self.add_state(
            "SAY_WELCOME",
            Say(format_str="Welcome to the party {}. Please follow me to be seated."),
            transitions={
                "succeeded": "STOP_EYE_TRACKING_1",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "STOP_EYE_TRACKING_1",
            StopEyeTracker(),
            transitions={
                "succeeded": "succeeded",
                "failed": "failed",
            },
        )

        self.add_state(
            "GET_ATTRIBUTE_STR",
            attribute,
            transitions={"succeeded": "SAY_ATTRIBUTE", "failed": "failed"},
        )

        self.add_state(
            "SAY_ATTRIBUTE",
            Say(),
            transitions={
                "succeeded": "STOP_EYE_TRACKING_2",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

        self.add_state(
            "STOP_EYE_TRACKING_2",
            StopEyeTracker(),
            transitions={
                "succeeded": "SAY_WELCOME_2",
                "failed": "failed",
            },
        )

        self.add_state(
            "SAY_WELCOME_2",
            Say(text="Please follow me to be seated."),
            transitions={
                "succeeded": "succeeded",
                "aborted": "failed",
                "canceled": "failed",
            },
        )

    def get_guest1_attributes(self, blackboard):
        attribute_str = ""
        attributes = blackboard["guest_data"]["guest1"]["attributes"]
        guest2_name = blackboard["guest_data"]["guest2"]["name"]
        guest1_name = blackboard["guest_data"]["guest1"]["name"]

        for attribute in attributes.keys():
            value = attributes[attribute]
            if attribute == "hair_color":
                attribute_str += f" have {value} coloured hair."
            elif attribute == "hair_length":
                attribute_str += f" have {value} hair."
            elif attribute == "glasses":
                attribute_str += (
                    " are wearing glasses." if value else " are not wearing glasses."
                )
            elif attribute == "hat":
                # attribute_str += (
                #     " are wearing a hat." if value else " are not wearing a hat."
                # )
                pass
            elif attribute == "shirt_color":
                attribute_str += f" are wearing a {value} coloured shirt."
            else:
                yasmin.YASMIN_LOG_ERROR(
                    f"The attribute {attribute} is not handled currently."
                )

        text = (
            f"Hello {guest2_name}, welcome to the party! {guest1_name} has already arrived and is sitting down. They "
            + attribute_str
        )
        yasmin.YASMIN_LOG_INFO(f"Attribute string: {text}")
        blackboard["text"] = text
        return "succeeded"
