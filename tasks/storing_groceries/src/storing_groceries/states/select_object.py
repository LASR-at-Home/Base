import smach

from storing_groceries.states import ChooseObject

class SelectObject(smach.StateMachine):
    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "failed"],
            input_keys=["table objects"],
        )

        with self:
            smach.StateMachine.add(
                "CHOOSE_OBJECT",
                ChooseObject(),
                transitions={
                    "succeeded": "CLASSIFY_CATEGORY_OBJECT",
                    "aborted": "CLASSIFY_CATEGORY_OBJECT",
                    "preempted": "CLASSIFY_CATEGORY_OBJECT",
                },
            )

            # smach.StateMachine.add(
            #     "MEASURE_OBJECT",
            #     Say(text="Measure object is ongoing"),
            #     transitions={
            #         "succeeded": "GRAB_OBJECT",
            #         "aborted": "GRAB_OBJECT",
            #         "preempted": "GRAB_OBJECT",
            #     },
            # )

            # smach.StateMachine.add(
            #     "SAY_MEASURE_OBJECT",
            #     Say(text="Measure object is ongoing"),
            #     transitions={
            #         "succeeded": "GO_TO_CABINET",
            #         "aborted": "GO_TO_CABINET",
            #         "preempted": "GO_TO_CABINET",
            #     },
            # )

            self.go_to_cabinet(self)
        