import smach

from storing_groceries.states import ChooseObject
from lasr_skills import Say


class SelectObject(smach.StateMachine):
    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "failed", "escape"],
            input_keys=["table_objects", "not_graspable"],
            output_keys=["table_object"],
        )

        with self:
            smach.StateMachine.add(
                "CHOOSE_OBJECT",
                ChooseObject(),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "succeeded",
                    "empty": "escape",  # should return escape in future
                },
            )
