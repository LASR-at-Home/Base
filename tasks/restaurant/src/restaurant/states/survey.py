import smach


class Survey(smach.StateMachine):

    def __init__(self) -> None:
        super().__init__(outcomes=["customer_found", "customer_not_found"])
