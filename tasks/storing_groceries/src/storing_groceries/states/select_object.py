import smach

from storing_groceries.states import ChooseObject
from lasr_skills import Say

class SelectObject(smach.StateMachine):
    def __init__(self):
        super().__init__(
            outcomes=["succeeded", "failed", "escape"],
            input_keys=["table_objects", "not_graspable"],
            output_keys=["table_object"]
        )

        with self:
            smach.StateMachine.add(
                "CHOOSE_OBJECT",
                ChooseObject(),
                transitions={
                    "succeeded": "MEASURE_OBJECT",
                    "failed": "MEASURE_OBJECT",
                    "empty": "escape", #should return escape in future 
                },

            )

            smach.StateMachine.add(
                "MEASURE_OBJECT",
                Say(text="Measure object is ongoing"),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "succeeded", #If fail then go to ChooseObject again
                    "preempted": "succeeded",
                },
            )
            
            

    
