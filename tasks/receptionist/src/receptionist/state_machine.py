from typing import List, Tuple, Dict

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Point, PointStamped, Pose
from lasr_skills import (
    GoToLocation,
    PlayMotion,
    Say,
    Wait,
    WaitForPersonInArea,
    Rotate,
    StopEyeTracker,
)
from receptionist.states import (
    HandleNameInterest,
    HandleDrink,
    SeatGuest,
    Introduce,
    WelcomeGuest,
    FindDrinkOnTable,
    GetCommonInterest,
    StartTimer,
    StopTimer,
    RecallDrinkOnTable,
)
from shapely.geometry import Polygon
from std_msgs.msg import Empty


class Receptionist(smach.StateMachine):
    def __init__(
        self,
        wait_pose: Pose,
        wait_area: Polygon,
        table_pose: Pose,
        table_point: Point,
        table_area: Polygon,
        table_left_area: Polygon,
        table_right_area: Polygon,
        centre_table_area: Polygon,
        seat_pose: Pose,
        seat_area: Polygon,
        sofa_area: Polygon,
        left_sofa_area: Polygon,
        right_sofa_area: Polygon,
        sofa_point: Point,
        host_data: Dict,
        max_people_on_sofa: int = 2,
        face_detection_confidence: float = 0.2,
        detect_drinks_once: bool = True,
    ):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        self.wait_pose = wait_pose
        self.wait_area = wait_area
        self.table_pose = table_pose
        self.table_area = table_area
        self.table_point = table_point
        self.left_table_area = table_left_area
        self.right_table_area = table_right_area
        self.centre_table_area = centre_table_area
        self.seat_pose = seat_pose
        self.seat_area = seat_area
        self.sofa_area = sofa_area
        self.left_sofa_area = left_sofa_area
        self.right_sofa_area = right_sofa_area
        self.sofa_point = sofa_point
        self.detect_drinks_once = detect_drinks_once

        prior_data: Dict[str, List[str]] = rospy.get_param("/receptionist/priors")
        self.possible_drinks = [drink.lower() for drink in prior_data["drinks"]]

        with self:
            self.userdata.guest_data = {
                "host": host_data,
                "guest1": {
                    "name": "",
                    "drink": "",
                    "interest": "",
                    "detection": False,
                    "seating_detection": False,
                },
                "guest2": {
                    "name": "",
                    "drink": "",
                    "interest": "",
                    "detection": False,
                    "seating_detection": False,
                },
            }
            drink_detections = {
                drink: {"detected": False, "location": "None"}
                for drink in self.possible_drinks
            }

            self.userdata.drink_detections = drink_detections
            self.userdata.confidence = face_detection_confidence
            self.userdata.dataset = "receptionist"
            self.userdata.drink_position = PointStamped()

            def wait_cb(ud, msg):
                rospy.loginfo("Received start signal")
                return False

            smach.StateMachine.add(
                "WAIT_START",
                smach_ros.MonitorState("/receptionist/start", Empty, wait_cb),
                transitions={
                    "valid": "WAIT_START",
                    "invalid": "START_TIMER",
                    "preempted": "WAIT_START",
                },
            )

            smach.StateMachine.add(
                "START_TIMER",
                StartTimer(),
                transitions={"succeeded": "START_CON", "failed": "START_CON"},
                remapping={"start_time": "start_time"},
            )

            start_con_sm = smach.Concurrence(
                outcomes=["succeeded", "failed"],
                default_outcome="failed",
                outcome_map={
                    "succeeded": {
                        "SAY_START": "succeeded",
                        "GO_TO_WAIT_LOCATION_GUEST_1": "succeeded",
                    },
                    "failed": {
                        "SAY_START": "aborted",
                        "GO_TO_WAIT_LOCATION_GUEST_1": "failed",
                    },
                },
            )
            with start_con_sm:
                smach.Concurrence.add(
                    "SAY_START", Say(text="Start of receptionist task.")
                )

                smach.Concurrence.add(
                    f"GO_TO_WAIT_LOCATION_GUEST_1", GoToLocation(self.wait_pose)
                )
            smach.StateMachine.add(
                "START_CON",
                start_con_sm,
                transitions={
                    "succeeded": "SAY_WAITING_GUEST_1",
                    "failed": "SAY_WAITING_GUEST_1",
                },
            )

            self._goto_waiting_area(guest_id=1)

            smach.StateMachine.add(
                "HANDLE_NAME_INTEREST_1",
                HandleNameInterest("guest1"),
                transitions={
                    "succeeded": "SAY_FOLLOW_GUEST_TO_TABLE_1",
                    "failed": "SAY_FOLLOW_GUEST_TO_TABLE_1",
                },
            )

            self._guide_guest_to_table(guest_id=1)

            smach.StateMachine.add(
                "HANDLE_FAVOURITE_DRINK_GUEST_1",
                HandleDrink("guest1"),
                transitions={
                    "succeeded": "FIND_DRINK_ON_TABLE_GUEST_1",
                    "failed": "FIND_DRINK_ON_TABLE_GUEST_1",
                },
            )

            smach.StateMachine.add(
                "FIND_DRINK_ON_TABLE_GUEST_1",
                FindDrinkOnTable(
                    guest_id="guest1",
                    table_point=self.table_point,
                    possible_drinks=self.possible_drinks,
                    table_area=self.table_area,
                    table_left_area=self.left_table_area,
                    table_right_area=self.right_table_area,
                    table_centre_area=self.centre_table_area,
                ),
                transitions={
                    "succeeded": "SAY_FOLLOW_GUEST_1",
                    "failed": "SAY_FOLLOW_GUEST_1",
                },
                remapping={
                    "guest_data": "guest_data",
                    "drink_location": "drink_position",
                },
            )

            self._guide_guest(guest_id=1)

            smach.StateMachine.add(
                "SEAT_GUEST_1",
                SeatGuest(
                    seating_area=self.seat_area,
                    sofa_area=self.sofa_area,
                    sofa_point=self.sofa_point,
                    left_sofa_area=self.left_sofa_area,
                    right_sofa_area=self.right_sofa_area,
                    max_people_on_sofa=max_people_on_sofa,
                    learn_host=True,
                ),
                transitions={
                    "succeeded": "INTRODUCE_GUEST_1",
                    "failed": "INTRODUCE_GUEST_1",
                },
                remapping={
                    "guest_seat_point": "guest_seat_point",
                    "seated_guest_locs": "seated_guest_locs",
                },
            )

            smach.StateMachine.add(
                "INTRODUCE_GUEST_1",
                Introduce(guest_to_introduce="guest1", can_detect_second_guest=False),
                transitions={"succeeded": "RETURN_CON", "failed": "RETURN_CON"},
            )

            return_con_sm = smach.Concurrence(
                outcomes=["succeeded", "failed"],
                default_outcome="failed",
                outcome_map={
                    "succeeded": {
                        "SAY_RETURN_WAITING_AREA": "succeeded",
                        "GO_TO_WAIT_LOCATION_GUEST_2": "succeeded",
                    },
                    "failed": {
                        "SAY_RETURN_WAITING_AREA": "aborted",
                        "GO_TO_WAIT_LOCATION_GUEST_2": "failed",
                    },
                },
            )
            with return_con_sm:
                smach.Concurrence.add(
                    "SAY_RETURN_WAITING_AREA",
                    Say(text="Let me go back to the waiting area."),
                )

                smach.Concurrence.add(
                    f"GO_TO_WAIT_LOCATION_GUEST_2", GoToLocation(self.wait_pose)
                )
            smach.StateMachine.add(
                "RETURN_CON",
                return_con_sm,
                transitions={
                    "succeeded": "SAY_WAITING_GUEST_2",
                    "failed": "SAY_WAITING_GUEST_2",
                },
            )

            self._goto_waiting_area(guest_id=2)

            smach.StateMachine.add(
                "HANDLE_NAME_INTEREST_2",
                HandleNameInterest("guest2"),
                transitions={
                    "succeeded": "WELCOME_GUEST_2",
                    "failed": "SAY_FOLLOW_GUEST_TO_TABLE_2",
                },
            )

            smach.StateMachine.add(
                "WELCOME_GUEST_2",
                WelcomeGuest(),
                transitions={
                    "succeeded": "SAY_FOLLOW_GUEST_TO_TABLE_2",
                    "failed": "SAY_FOLLOW_GUEST_TO_TABLE_2",
                },
            )

            self._guide_guest_to_table(guest_id=2)

            smach.StateMachine.add(
                "HANDLE_FAVOURITE_DRINK_GUEST_2",
                HandleDrink("guest2", face_table=(not self.detect_drinks_once)),
                transitions={
                    "succeeded": "FIND_DRINK_ON_TABLE_GUEST_2",
                    "failed": "FIND_DRINK_ON_TABLE_GUEST_2",
                },
            )

            if self.detect_drinks_once:
                smach.StateMachine.add(
                    "FIND_DRINK_ON_TABLE_GUEST_2",
                    RecallDrinkOnTable(guest_id="guest2"),
                    transitions={
                        "succeeded": "SAY_FOLLOW_GUEST_2",
                        "failed": "SAY_FOLLOW_GUEST_2",
                    },
                    remapping={
                        "guest_data": "guest_data",
                        "drink_location": "drink_position",
                    },
                )
            else:
                smach.StateMachine.add(
                    "FIND_DRINK_ON_TABLE_GUEST_2",
                    FindDrinkOnTable(
                        guest_id="guest2",
                        table_point=self.table_point,
                        possible_drinks=self.possible_drinks,
                        table_area=self.table_area,
                        table_left_area=self.left_table_area,
                        table_right_area=self.right_table_area,
                        table_centre_area=self.centre_table_area,
                    ),
                    transitions={
                        "succeeded": "SAY_FOLLOW_GUEST_2",
                        "failed": "SAY_FOLLOW_GUEST_2",
                    },
                    remapping={
                        "guest_data": "guest_data",
                        "drink_location": "drink_position",
                    },
                )

            self._guide_guest(guest_id=2)

            smach.StateMachine.add(
                "SEAT_GUEST_2",
                SeatGuest(
                    seating_area=self.seat_area,
                    sofa_area=self.sofa_area,
                    sofa_point=self.sofa_point,
                    left_sofa_area=self.left_sofa_area,
                    right_sofa_area=self.right_sofa_area,
                    max_people_on_sofa=max_people_on_sofa,
                ),
                transitions={
                    "succeeded": "INTRODUCE_COMMON_INTEREST_CON",
                    "failed": "INTRODUCE_COMMON_INTEREST_CON",
                },
                remapping={
                    "guest_seat_point": "guest_seat_point",
                    "seated_guest_locs": "seated_guest_locs",
                },
            )
            sm_con = smach.Concurrence(
                outcomes=["succeeded", "failed"],
                input_keys=["guest_data", "seated_guest_locs", "guest_seat_point"],
                output_keys=["interest_message"],
                default_outcome="failed",
                outcome_map={
                    "succeeded": {
                        "INTRODUCE_GUEST_2": "succeeded",
                        "GET_COMMON_INTEREST": "succeeded",
                    },
                    "failed": {
                        "INTRODUCE_GUEST_2": "failed",
                        "GET_COMMON_INTEREST": "failed",
                    },
                },
            )
            with sm_con:
                smach.Concurrence.add(
                    "INTRODUCE_GUEST_2",
                    Introduce(
                        guest_to_introduce="guest2", can_detect_second_guest=True
                    ),
                )

                smach.Concurrence.add("GET_COMMON_INTEREST", GetCommonInterest())
            smach.StateMachine.add(
                "INTRODUCE_COMMON_INTEREST_CON",
                sm_con,
                transitions={"succeeded": "SAY_INTEREST", "failed": "SAY_INTEREST"},
                remapping={
                    "guest_data": "guest_data",
                    "interest_message": "interest_message",
                },
            )
            smach.StateMachine.add(
                "SAY_INTEREST",
                Say(),
                transitions={
                    "succeeded": "SAY_GOODBYE",
                    "aborted": "SAY_GOODBYE",
                    "preempted": "SAY_GOODBYE",
                },
                remapping={"text": "interest_message"},
            )

            """
            Finish
            """
            smach.StateMachine.add(
                "SAY_GOODBYE",
                Say(text="Enjoy the party!"),
                transitions={
                    "succeeded": "SAY_FINISHED",
                    "aborted": "SAY_FINISHED",
                    "preempted": "SAY_FINISHED",
                },
            )
            smach.StateMachine.add(
                "SAY_FINISHED",
                Say(text="I am done."),
                transitions={
                    "succeeded": "STOP_TIMER",
                    "aborted": "STOP_TIMER",
                    "preempted": "STOP_TIMER",
                },
            )
            smach.StateMachine.add(
                "STOP_TIMER",
                StopTimer(),
                transitions={"succeeded": "SAY_TIME", "failed": "failed"},
            )
            smach.StateMachine.add(
                "SAY_TIME",
                Say(),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"text": "time_text"},
            )

    def _goto_waiting_area(self, guest_id: int) -> None:
        """Adds the states to go to the waiting area.

        Args:
            guest_id (int): Identifier for the guest.
        """

        smach.StateMachine.add(
            f"SAY_WAITING_GUEST_{guest_id}",
            Say(text="I am waiting for a guest."),
            transitions={
                "succeeded": f"WAIT_FOR_PERSON_GUEST_{guest_id}",
                "aborted": f"WAIT_FOR_PERSON_GUEST_{guest_id}",
                "preempted": f"WAIT_FOR_PERSON_GUEST_{guest_id}",
            },
        )

        smach.StateMachine.add(
            f"WAIT_FOR_PERSON_GUEST_{guest_id}",
            WaitForPersonInArea(self.wait_area),
            transitions={
                "succeeded": f"HANDLE_NAME_INTEREST_{guest_id}",
                "failed": f"HANDLE_NAME_INTEREST_{guest_id}",
            },
            remapping={"detections_3d": "person_detections"},
        )

    def _guide_guest_to_table(self, guest_id: int) -> None:
        """Adds the states to guide a guest to the
        seating area.

        Args:
            guest_id (int): Identifier for the guest.
        """

        smach.StateMachine.add(
            f"SAY_FOLLOW_GUEST_TO_TABLE_{guest_id}",
            Say(text="Please follow me, I will guide you to the beverage area"),
            transitions={
                "succeeded": f"STOP_EYE_TRACKER_GUEST_{guest_id}",
                "preempted": f"STOP_EYE_TRACKER_GUEST_{guest_id}",
                "aborted": f"STOP_EYE_TRACKER_GUEST_{guest_id}",
            },
        )

        smach.StateMachine.add(
            f"STOP_EYE_TRACKER_GUEST_{guest_id}",
            StopEyeTracker(),
            transitions={
                "succeeded": f"GO_TO_TABLE_LOCATION_GUEST_{guest_id}",
                "failed": f"GO_TO_TABLE_LOCATION_GUEST_{guest_id}",
            },
        )

        smach.StateMachine.add(
            f"GO_TO_TABLE_LOCATION_GUEST_{guest_id}",
            GoToLocation(self.table_pose),
            transitions={
                "succeeded": f"SAY_ARRIVE_GUEST_{guest_id}",
                "failed": f"GO_TO_TABLE_LOCATION_GUEST_{guest_id}",
            },
        )

        smach.StateMachine.add(
            f"SAY_ARRIVE_GUEST_{guest_id}",
            Say(text="This is the beverage area."),
            transitions={
                "succeeded": f"HANDLE_FAVOURITE_DRINK_GUEST_{guest_id}",
                "preempted": f"HANDLE_FAVOURITE_DRINK_GUEST_{guest_id}",
                "aborted": f"HANDLE_FAVOURITE_DRINK_GUEST_{guest_id}",
            },
        )

    def _guide_guest(self, guest_id: int) -> None:
        """Adds the states to guide a guest to the
        seating area.

        Args:
            guest_id (int): Identifier for the guest.
        """

        smach.StateMachine.add(
            f"SAY_FOLLOW_GUEST_{guest_id}",
            Say(text="Please follow me, I will guide you to the other guests"),
            transitions={
                "succeeded": f"GO_TO_SEAT_LOCATION_GUEST_{guest_id}",
                "preempted": "failed",
                "aborted": "failed",
            },
        )

        smach.StateMachine.add(
            f"GO_TO_SEAT_LOCATION_GUEST_{guest_id}",
            GoToLocation(self.seat_pose),
            transitions={
                "succeeded": f"SEAT_GUEST_{guest_id}",
                "failed": f"GO_TO_SEAT_LOCATION_GUEST_{guest_id}",
            },
        )
