import smach
from .states import (
    TakeOrder,
    MakeOrder,
    CheckOrder,
    GoToTable,
    DeliverOrder,
    GoToCounter,
    Start,
    LoadOrder,
    WaitForOrder,
)


class Phase2(smach.StateMachine):
    def __init__(self, context):
        smach.StateMachine.__init__(self, outcomes=["done"])

        with self:
            smach.StateMachine.add(
                "START_PHASE_2", Start(context), transitions={"done": "GO_TO_TABLE"}
            )
            smach.StateMachine.add(
                "GO_TO_TABLE",
                GoToTable(context),
                transitions={"done": "TAKE_ORDER", "skip": "done"},
            )
            smach.StateMachine.add(
                "TAKE_ORDER", TakeOrder(context), transitions={"done": "GO_TO_COUNTER"}
            )
            smach.StateMachine.add(
                "GO_TO_COUNTER",
                GoToCounter(context),
                transitions={"done": "MAKE_ORDER"},
            )
            smach.StateMachine.add(
                "MAKE_ORDER", MakeOrder(context), transitions={"done": "WAIT_FOR_ORDER"}
            )
            smach.StateMachine.add(
                "WAIT_FOR_ORDER",
                WaitForOrder(context),
                transitions={"done": "CHECK_ORDER"},
            )
            smach.StateMachine.add(
                "CHECK_ORDER",
                CheckOrder(context),
                transitions={"correct": "LOAD_ORDER", "incorrect": "WAIT_FOR_ORDER"},
            )
            smach.StateMachine.add(
                "LOAD_ORDER", LoadOrder(context), transitions={"done": "DELIVER_ORDER"}
            )

            @smach.cb_interface(input_keys=[], output_keys=[], outcomes=["done"])
            def reset_tables(ud):
                for table in context.tables.keys():
                    context.tables[table]["status"] = "unvisited"
                return "done"

            smach.StateMachine.add(
                "DELIVER_ORDER",
                DeliverOrder(context),
                transitions={"done": "RESET_TABLES"},
            )

            smach.StateMachine.add(
                "RESET_TABLES",
                smach.CBState(reset_tables),
                transitions={"done": "done"},
            )
