import yasmin
import yasmin_ros
from std_msgs.msg import Empty
from lasr_skills import Say
from lasr_skills.start_task import StartDoorSM
from lasr_skills import Say, DetectDoorOpening, GoToLocation

class Start(yasmin.StateMachine):
    """
    Entry sequence for the Doing Laundry task.

    Sequence:
        1. Wait for start signal on /doing_laundry/start
        2. Say "Start of Doing Laundry task"
        3. Say "Waiting for the door to open"
        4. Wait for door to open, then navigate to the laundry area
           (handled by StartDoorSM)

    Blackboard outputs:
        (none — navigation target loaded from params)
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
                topic_name="/doing_laundry/start",
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
            Say(text="Start of Doing Laundry task."),
            transitions={
                "succeeded": "SAY_WAITING",
                "aborted": "SAY_WAITING",
                "canceled": "SAY_WAITING",
            },
        )

        self.add_state(
            "SAY_WAITING",
            Say(text="Waiting for the door to open."),
            transitions={
                "succeeded": "WAIT_FOR_DOOR",
                "aborted": "WAIT_FOR_DOOR",
                "canceled": "WAIT_FOR_DOOR",
            },
        )
        # 4. Detect door opening — long timeout for testing
        self.add_state(
            "WAIT_FOR_DOOR",
            DetectDoorOpening(lasr_scan_topic="/scan_raw", timeout=999.0),
            transitions={
                "door_opened": "GO_TO_LAUNDRY_AREA",
                "failed": "WAIT_FOR_DOOR",
            },
        )

        # 5. Navigate to laundry area
        self.add_state(
            "GO_TO_LAUNDRY_AREA",
            GoToLocation(location_param="laundry_area.pose"),
            transitions={
                "succeeded": "succeeded",
                "failed": "failed",
            },
        )