import smach
from lasr_skills import Say
from receptionist.states import RecogniseDrink, DetermineDrinkPosition

class HandleFavouriteDrink(smach.StateMachine):
    """
    1) RecogniseDrink → available/not_available
    2) DetermineDrinkPosition → left/right/centre
    3) Say the result
    """
    def __init__(self, guest_key: str, dataset: str, confidence: float):
        super().__init__(
            outcomes=["succeeded","failed"],
            input_keys=["guest_data"],
            output_keys=["guest_data"],
        )
        self._dataset    = dataset
        self._confidence = confidence
        
        with self:
           
            smach.StateMachine.add(
                "RECOGNISE_DRINK",
                RecogniseDrink(self._dataset, self._confidence),
                transitions={
                    "available":     "DETERMINE_DRINK_POSITION",
                    "not_available": "SAY_NOT_AVAILABLE",
                    "failed":        "SAY_NOT_AVAILABLE",
                },
            )

            smach.StateMachine.add(
                "DETERMINE_DRINK_POSITION",
                DetermineDrinkPosition("/receptionist/table_area"),
                transitions={
                    "succeeded":"SAY_DRINK_POSITION",
                    "failed":   "SAY_NOT_AVAILABLE",
                },
            )

            smach.StateMachine.add(
                "SAY_DRINK_POSITION",
                Say(format_str="Your drink is on the {} of the table."),
                remapping={"placeholders":"drink_position_str"},
                transitions={"succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "succeeded",},
            )

            smach.StateMachine.add(
                "SAY_NOT_AVAILABLE",
                Say(text="Unfortunately, your favourite drink is not available."),
                transitions={"succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "succeeded",},
            )
