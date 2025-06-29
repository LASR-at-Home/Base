import smach

from lasr_skills import (
    AskAndListen,
    GetNearestObject,
    Detect3D,
    StartEyeTracker,
    StopEyeTracker,
    Rotate,
    PlayMotion,
    Say,
)
from receptionist.states import (
    GetDrink,
)


class HandleDrink(smach.StateMachine):
    def __init__(self, guest_id: str):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["guest_data"],
            output_keys=["guest_data"],
        )

        with self:
            # Detect the nearest person
            smach.StateMachine.add(
                f"LOOK_CENTRE_BEVERAGE_GUEST_{guest_id}",
                PlayMotion("look_centre"),
                transitions={
                    "succeeded": f"DETECT_PERSON_{guest_id}",
                    "preempted": f"DETECT_PERSON_{guest_id}",
                    "aborted": f"DETECT_PERSON_{guest_id}",
                },
            )

            smach.StateMachine.add(
                f"DETECT_PERSON_{guest_id}",
                Detect3D(
                    filter=["person"],
                    confidence=0.5,
                    target_frame="map",
                ),
                transitions={
                    "succeeded": f"GET_NEAREST_PERSON_{guest_id}",
                    "failed": f"GET_DRINK_GUEST_{guest_id}",
                },
                remapping={
                    "detections_3d": "detections_3d",
                    "image_raw": "image_raw",
                },
            )
            smach.StateMachine.add(
                f"GET_NEAREST_PERSON_{guest_id}",
                GetNearestObject(),
                transitions={
                    "succeeded": f"START_EYE_TRACKER_GUEST_{guest_id}",
                    "failed": f"GET_DRINK_GUEST_{guest_id}",
                },
                remapping={
                    "nearest_object": "person_point",
                },
            )
            # If there, begin eye tracker, else, say can't se ebut trust you are there
            smach.StateMachine.add(
                f"START_EYE_TRACKER_GUEST_{guest_id}",
                StartEyeTracker(),
                transitions={
                    "succeeded": f"GET_DRINK_GUEST_{guest_id}",
                    "failed": f"GET_DRINK_GUEST_{guest_id}",
                },
                remapping={"person_point": "person_point"},
            )
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
                    "succeeded": f"STOP_EYE_TRACKER_GUEST_{guest_id}",
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
                    "failed": f"STOP_EYE_TRACKER_GUEST_{guest_id}",
                },
            )

            smach.StateMachine.add(
                f"REPEAT_PARSE_DRINK_GUEST_{guest_id}",
                GetDrink(guest_id, True),
                transitions={
                    "succeeded": f"STOP_EYE_TRACKER_GUEST_{guest_id}",
                    "failed": f"STOP_EYE_TRACKER_GUEST_{guest_id}",
                },
                remapping={"guest_transcription": "transcribed_speech"},
            )

            smach.StateMachine.add(
                f"STOP_EYE_TRACKER_GUEST_{guest_id}",
                StopEyeTracker(),
                transitions={
                    "succeeded": f"SAY_FIND_DRINK_{guest_id}",
                    "failed": f"SAY_FIND_DRINK_{guest_id}",
                },
            )
            smach.StateMachine.add(
                f"SAY_FIND_DRINK_{guest_id}",
                Say("Thank you! I will find your drink now."),
                transitions={
                    "succeeded": f"GET_DRINK_GUEST_{guest_id}",
                    "failed": f"GET_DRINK_GUEST_{guest_id}",
                },
            )
            smach.StateMachine.add(
                f"FACE_TABLE_GUEST_{guest_id}",
                Rotate(180),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )
