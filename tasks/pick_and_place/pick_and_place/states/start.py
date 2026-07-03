import yasmin
import yasmin_ros

from std_msgs.msg import Empty
from lasr_skills import Say, GoToLocation, StartDoorSM


class Start(yasmin.StateMachine):
    """
    Entry sequence for the Pick and Place task.

    Ported from ROS 1 SMACH Start. The five-state sequence collapses
    into a YASMIN StateMachine using lasr_skills states directly.

    Sequence:
        1. Wait for start signal on /pick_and_place/start
        2. Say "Start of Pick and Place task"
        3. Say "Waiting for the door to open"
        4. Detect door opening
        5. Navigate to the table
        6. Ask referee to open cabinet doors

    Blackboard outputs:
        (none — all navigation targets loaded from params)
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"], handle_sigint=True)

        # 1. Wait for start signal
        def wait_cb(blackboard, msg):
            yasmin.YASMIN_LOG_INFO("Received start signal.")
            return "succeeded"

        self.add_state(
            "WAIT_START",
            yasmin_ros.MonitorState(
                topic_name="/pick_and_place/start",
                msg_type=Empty,
                monitor_handler=wait_cb,
                outcomes=["succeeded", "failed"],
            ),
            transitions={
                "succeeded": "SAY_START",
                "failed": "WAIT_START",
                "canceled": "failed",
            },
        )

        # 2. Announce start
        self.add_state(
            "SAY_START",
            Say(text="Start of Pick and Place task."),
            transitions={
                "succeeded": "WAIT_FOR_DOOR",
                "aborted": "WAIT_FOR_DOOR",
                "canceled": "WAIT_FOR_DOOR",
            },
        )

        # 4. Detect door opening
        self.add_state(
            "WAIT_FOR_DOOR",
            StartDoorSM(),
            transitions={
                "succeeded": "SAY_GOING_TO_TABLE",
                "failed": "SAY_GOING_TO_TABLE",  # FIX THIS ON THE REAL ROBOT
            },
        )

        # 5. Announce navigatGO_TO_TRASH_BIN_FLOORion
        self.add_state(
            "SAY_GOING_TO_TABLE",
            Say(text="I am going to the table."),
            transitions={
                "succeeded": "GO_TO_TABLE",
                "aborted": "GO_TO_TABLE",
                "canceled": "GO_TO_TABLE",
            },
        )

        # # 6. Navigate to table
        self.add_state(
            "GO_TO_TABLE",
            GoToLocation(location_param="pick_and_place.table.pose"),
            transitions={
                "succeeded": "ASK_OPEN_CABINET",
                "failed": "ASK_OPEN_CABINET",
            },
        )

        # 7. Ask referee to open cabinet
        self.add_state(
            "ASK_OPEN_CABINET",
            Say(text=""),
            transitions={
                "succeeded": "succeeded",
                "aborted": "succeeded",
                "canceled": "succeeded",
            },
        )
