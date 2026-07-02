import yasmin
from lasr_skills import Say, PlayMotion, Rotate


class LookForBasket(yasmin.StateMachine):
    """
    State machine for searching for the laundry basket.

    Sequence:
        1. Say "I am looking for the laundry basket"
        2. Rotate 180 degrees to face first half of room
        3. Sweep head left → check VLM → centre → check VLM → right → check VLM
        4. Rotate 180 degrees to face second half of room
        5. Sweep head left → check VLM → centre → check VLM → right → check VLM
        6. Say "I have finished searching for the basket"

    Outcomes:
        found     — basket detected (stub: never triggered yet)
        not_found — sweep complete, basket not found

    Future:
        Replace CHECK_BASKET_* states with real DetectBasketWithVLM()
        which calls /vlm/query with image + prompt and returns found/not_found
    """

    def __init__(self):
        super().__init__(outcomes=["found", "not_found"], handle_sigint=True)

        # 1. Announce searching
        self.add_state(
            "SAY_SEARCHING",
            Say(text="I am looking for the laundry basket."),
            transitions={
                "succeeded": "ROTATE_180_FIRST",
                "aborted": "ROTATE_180_FIRST",
                "canceled": "ROTATE_180_FIRST",
            },
        )

        # 2. Rotate 180 to face first half of room
        self.add_state(
            "ROTATE_180_FIRST",
            Rotate(angle=180),
            transitions={
                "succeeded": "LOOK_LEFT_1",
                "failed": "LOOK_LEFT_1",
            },
        )

        # 3. Head sweep — first half with VLM stubs
        self.add_state(
            "LOOK_LEFT_1",
            PlayMotion(motion_name="look_left"),
            transitions={
                "succeeded": "CHECK_BASKET_LEFT_1",
                "aborted": "CHECK_BASKET_LEFT_1",
                "canceled": "CHECK_BASKET_LEFT_1",
            },
        )

        # STUB — replace with DetectBasketWithVLM()
        self.add_state(
            "CHECK_BASKET_LEFT_1",
            Say(text="Checking for basket on the left."),
            transitions={
                "succeeded": "LOOK_CENTRE_1",
                "aborted": "LOOK_CENTRE_1",
                "canceled": "LOOK_CENTRE_1",
            },
        )

        self.add_state(
            "LOOK_CENTRE_1",
            PlayMotion(motion_name="look_centre"),
            transitions={
                "succeeded": "CHECK_BASKET_CENTRE_1",
                "aborted": "CHECK_BASKET_CENTRE_1",
                "canceled": "CHECK_BASKET_CENTRE_1",
            },
        )

        # STUB — replace with DetectBasketWithVLM()
        self.add_state(
            "CHECK_BASKET_CENTRE_1",
            Say(text="Checking for basket in the centre."),
            transitions={
                "succeeded": "LOOK_RIGHT_1",
                "aborted": "LOOK_RIGHT_1",
                "canceled": "LOOK_RIGHT_1",
            },
        )

        self.add_state(
            "LOOK_RIGHT_1",
            PlayMotion(motion_name="look_right"),
            transitions={
                "succeeded": "CHECK_BASKET_RIGHT_1",
                "aborted": "CHECK_BASKET_RIGHT_1",
                "canceled": "CHECK_BASKET_RIGHT_1",
            },
        )

        # STUB — replace with DetectBasketWithVLM()
        self.add_state(
            "CHECK_BASKET_RIGHT_1",
            Say(text="Checking for basket on the right."),
            transitions={
                "succeeded": "ROTATE_180_SECOND",
                "aborted": "ROTATE_180_SECOND",
                "canceled": "ROTATE_180_SECOND",
            },
        )

        # 4. Rotate 180 to face second half of room
        self.add_state(
            "ROTATE_180_SECOND",
            Rotate(angle=180),
            transitions={
                "succeeded": "LOOK_LEFT_2",
                "failed": "LOOK_LEFT_2",
            },
        )

        # 5. Head sweep — second half with VLM stubs
        self.add_state(
            "LOOK_LEFT_2",
            PlayMotion(motion_name="look_left"),
            transitions={
                "succeeded": "CHECK_BASKET_LEFT_2",
                "aborted": "CHECK_BASKET_LEFT_2",
                "canceled": "CHECK_BASKET_LEFT_2",
            },
        )

        # STUB — replace with DetectBasketWithVLM()
        self.add_state(
            "CHECK_BASKET_LEFT_2",
            Say(text="Checking for basket on the left."),
            transitions={
                "succeeded": "LOOK_CENTRE_2",
                "aborted": "LOOK_CENTRE_2",
                "canceled": "LOOK_CENTRE_2",
            },
        )

        self.add_state(
            "LOOK_CENTRE_2",
            PlayMotion(motion_name="look_centre"),
            transitions={
                "succeeded": "CHECK_BASKET_CENTRE_2",
                "aborted": "CHECK_BASKET_CENTRE_2",
                "canceled": "CHECK_BASKET_CENTRE_2",
            },
        )

        # STUB — replace with DetectBasketWithVLM()
        self.add_state(
            "CHECK_BASKET_CENTRE_2",
            Say(text="Checking for basket in the centre."),
            transitions={
                "succeeded": "LOOK_RIGHT_2",
                "aborted": "LOOK_RIGHT_2",
                "canceled": "LOOK_RIGHT_2",
            },
        )

        self.add_state(
            "LOOK_RIGHT_2",
            PlayMotion(motion_name="look_right"),
            transitions={
                "succeeded": "CHECK_BASKET_RIGHT_2",
                "aborted": "CHECK_BASKET_RIGHT_2",
                "canceled": "CHECK_BASKET_RIGHT_2",
            },
        )

        # STUB — replace with DetectBasketWithVLM()
        self.add_state(
            "CHECK_BASKET_RIGHT_2",
            Say(text="Checking for basket on the right."),
            transitions={
                "succeeded": "LOOK_CENTRE_FINAL",
                "aborted": "LOOK_CENTRE_FINAL",
                "canceled": "LOOK_CENTRE_FINAL",
            },
        )

        # 6. Return head to centre
        self.add_state(
            "LOOK_CENTRE_FINAL",
            PlayMotion(motion_name="look_centre"),
            transitions={
                "succeeded": "SAY_DONE",
                "aborted": "SAY_DONE",
                "canceled": "SAY_DONE",
            },
        )

        # 7. Announce done
        self.add_state(
            "SAY_DONE",
            Say(text="I could not find the laundry basket."),
            transitions={
                "succeeded": "not_found",
                "aborted": "not_found",
                "canceled": "not_found",
            },
        )