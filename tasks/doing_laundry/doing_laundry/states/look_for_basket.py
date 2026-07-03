import yasmin
from lasr_skills import Say, PlayMotion, Rotate
from doing_laundry.states.detect_basket import DetectBasket


class LookForBasket(yasmin.StateMachine):
    """
    State machine for searching for the laundry basket.

    Sequence:
        1. Say "I am looking for the laundry basket"
        2. Rotate 180 degrees to face first half of room
        3. Sweep head left → detect → centre → detect → right → detect
        4. Rotate 180 degrees to face second half of room
        5. Sweep head left → detect → centre → detect → right → detect
        6. Say "I could not find the laundry basket"

    Outcomes:
        found     — basket detected by open vocab
        not_found — sweep complete, basket not found
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

        # 2. Rotate 180 first
        self.add_state(
            "ROTATE_180_FIRST",
            Rotate(angle=180),
            transitions={
                "succeeded": "LOOK_LEFT_1",
                "failed": "LOOK_LEFT_1",
            },
        )

        # 3. Head sweep — first half
        self.add_state(
            "LOOK_LEFT_1",
            PlayMotion(motion_name="look_left"),
            transitions={
                "succeeded": "CHECK_BASKET_LEFT_1",
                "aborted": "CHECK_BASKET_LEFT_1",
                "canceled": "CHECK_BASKET_LEFT_1",
            },
        )

        self.add_state(
            "CHECK_BASKET_LEFT_1",
            DetectBasket(),
            transitions={
                "found": "found",
                "not_found": "LOOK_CENTRE_1",
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

        self.add_state(
            "CHECK_BASKET_CENTRE_1",
            DetectBasket(),
            transitions={
                "found": "found",
                "not_found": "LOOK_RIGHT_1",
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

        self.add_state(
            "CHECK_BASKET_RIGHT_1",
            DetectBasket(),
            transitions={
                "found": "found",
                "not_found": "SAY_DONE",
            },
        )

        # 4. Rotate 180 second
        self.add_state(
            "ROTATE_180_SECOND",
            Rotate(angle=180),
            transitions={
                "succeeded": "LOOK_LEFT_2",
                "failed": "LOOK_LEFT_2",
            },
        )

        # 5. Head sweep — second half
        self.add_state(
            "LOOK_LEFT_2",
            PlayMotion(motion_name="look_left"),
            transitions={
                "succeeded": "CHECK_BASKET_LEFT_2",
                "aborted": "CHECK_BASKET_LEFT_2",
                "canceled": "CHECK_BASKET_LEFT_2",
            },
        )

        self.add_state(
            "CHECK_BASKET_LEFT_2",
            DetectBasket(),
            transitions={
                "found": "found",
                "not_found": "LOOK_CENTRE_2",
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

        self.add_state(
            "CHECK_BASKET_CENTRE_2",
            DetectBasket(),
            transitions={
                "found": "found",
                "not_found": "LOOK_RIGHT_2",
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

        self.add_state(
            "CHECK_BASKET_RIGHT_2",
            DetectBasket(),
            transitions={
                "found": "found",
                "not_found": "LOOK_CENTRE_FINAL",
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