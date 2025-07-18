import rospy
import smach
from smach import CBState
from lasr_skills import Say, AdjustCamera, GoToLocation, CheckDoorStatus, DetectDict
from give_me_a_hand.states import (
    Survey, HandleOrder, GetPoses
)

import smach
import smach_ros
from geometry_msgs.msg import Pose
from lasr_skills import AskAndListen, GoToLocation, Rotate, Say, Wait
# from std_msgs.msg import Empty


# def is_cabinet_checked_cb(userdata):
#     return "true" if userdata.all_cabinet_open else "false"


# def set_cabinet_checked_cb(userdata):
#     userdata.all_cabinet_open = True
#     return "done"


class FindOperators(smach.StateMachine):
    def __init__(self, table_pose, cabinet_pose):
        super().__init__(
            outcomes=["succeeded", "failed", "escape"],
            input_keys=[],
            output_keys=[],
        )

        with self:
            self.go_to_livingroom()

            smach.StateMachine.add(
                "ROTATE_360",
                Rotate(angle=360.0),
                transitions={"succeeded": "GET_POSES", "failed": "ROTATE_360"},
            )

            smach.StateMachine.add(
                "GET_POSES", GetPoses(), transitions={"succeeded": "SURVEY"}
            )

            smach.StateMachine.add(
                "SURVEY",
                Survey((-71.0, 71.0), 10),
                transitions={
                    "customer_found": "GO_TO_OPERATORS",
                    "customer_not_found": "escape",
                },
            )

            smach.StateMachine.add(
                "GO_TO_OPERATORS",
                GoToLocation(),
                remapping={"location": "customer_approach_pose"},
                transitions={"succeeded": "GET_ORDER", "failed": "TRUN_MOVE_BASE"},
            )

            smach.StateMachine.add(
                "GET_ORDER",
                GetOrder(),
                transitions={
                    "succeeded": "HOLD_OBJECT",
                    "failed": "HOLD_OBJECT",
                },
                remapping={"detections": "table_objects"},
            )

            smach.StateMachine.add(
                "HOLD_OBJECT",
                HoldObject(),
                transitions={
                    "succeeded": "DELIVER_OBJECT",
                    "aborted": "DELIVER_OBJECT",
                    "preempted": "DELIVER_OBJECT",
                },
            )

            smach.StateMachine.add(
                "DELIVER_OBJECT",
                DeliverObject(),
                transitions={
                    "succeeded": "SELECT_OBJECT",
                    "aborted": "SELECT_OBJECT",
                    
                },
            )

            smach.StateMachine.add(
                "PUT_OBJECT",
                PutObject(),
                transitions={
                    "succeeded": "SAY_ORDER_DONE",
                    "aborted": "SAY_ORDER_DONE",
                    "preempted": "SAY_ORDER_DONE",
                },
            )

            smach.StateMachine.add(
                "SAY_ORDER_DONE",
                Say(text="Order is done"),
                transitions={
                    "succeeded": "CLASSIFY_CATEGORY_OBJECT",
                    "aborted": "CLASSIFY_CATEGORY_OBJECT",
                    "preempted": "CLASSIFY_CATEGORY_OBJECT",
                },
            )

    def go_to_(self, pose, next_state) -> None:
        """Adds the states to go to table area."""

        smach.StateMachine.add(
            f"GO_TO_{pose}",
            GoToLocation(pose),
            transitions={
                "succeeded": f"SAY_ARRIVE_TABLE",
                "failed": f"GO_TO_RECOVERY",
            },
        )

        smach.StateMachine.add(
            f"GO_TO_RECOVERY",
            GoToRecovery(pose),
            transitions={
                "succeeded": f"SAY_DETECT_TABLE",
                "failed": f"{next_state}",
            },
        )

    def go_to_cabinet(self) -> None:
        """Adds the states to go to cabinet area."""
        smach.StateMachine.add(
                "GO_TO_RECOVERY",
                GoToRecovery(),
                remapping={"location": "customer_approach_pose"},
                transitions={"succeeded": "GO_TO_OPERATORS", "failed": "failed"}, #shell I make it fail??
            ) #recovery for every thing??

        smach.StateMachine.add(
            "GO_TO_CABINET_SHELF",
            GoToCabinetShelfDynamic(self.cabinet_pose),
            transitions={
                "succeeded": "SAY_ARRIVE_CABINET_SHELF",
                "failed": "SAY_ARRIVE_CABINET_SHELF",
            },
        )

        smach.StateMachine.add(
            f"SAY_ARRIVE_CABINET",
            Say(text="Arrive cabinet"),
            transitions={
                "succeeded": f"CHECK_IF_CABINET_ALREADY_OPEN",
                "aborted": f"CHECK_IF_CABINET_ALREADY_OPEN",
                "preempted": f"CHECK_IF_CABINET_ALREADY_OPEN",
            },
        )

    def go_to_cabinet_shelf(self) -> None:
        class GoToCabinetShelfDynamic(smach.State):
            def __init__(self, poses):
                smach.State.__init__(
                    self, outcomes=["succeeded", "failed"], input_keys=["cabinet_num"]
                )
                self.poses = poses

            def execute(self, userdata):
                pose_index = userdata.cabinet_num
                rospy.loginfo(
                    f"[GoToCabinetShelfDynamic] Going to cabinet shelf {pose_index}"
                )
                goto = GoToLocation(self.poses[pose_index])
                return goto.execute(userdata)

        smach.StateMachine.add(
            "GO_TO_CABINET_SHELF",
            GoToCabinetShelfDynamic(self.cabinet_pose),
            transitions={
                "succeeded": "SAY_ARRIVE_CABINET_SHELF",
                "failed": "SAY_ARRIVE_CABINET_SHELF",
            },
        )

        smach.StateMachine.add(
            "SAY_ARRIVE_CABINET_SHELF",
            Say(text="Arrive cabinet shelf"),
            transitions={
                "succeeded": "PUT_OBJECT",
                "aborted": "PUT_OBJECT",
                "preempted": "PUT_OBJECT",
            },
        )

    def check_cabinet(self) -> None:
        """Adds the states to check and classify each cabinet pose once, using CBState for runtime branching."""

        smach.StateMachine.add(
            "CHECK_IF_CABINET_ALREADY_OPEN",
            CBState(
                is_cabinet_checked_cb,
                outcomes=["true", "false"],
                input_keys=["all_cabinet_open"],
            ),
            transitions={
                "true": "SAY_CABINET_ALREADY_CHECKED",
                "false": "CHECK_CABINET",
            },
        )

        smach.StateMachine.add(
            "SAY_CABINET_ALREADY_CHECKED",
            Say(text="Cabinet already checked"),
            transitions={
                "succeeded": "SAY_CABINET_CATEGORY",
                "aborted": "SAY_CABINET_CATEGORY",
                "preempted": "SAY_CABINET_CATEGORY",
            },
        )

        # Sub-state machine to loop through each cabinet pose
        check_cabinet_sm = smach.StateMachine(
            outcomes=["done"],
            input_keys=[
                "all_cabinet_open",
                "cabinet_objects",
                "cabinets_objects",
                "all_cabinet_open",
                "table_object",
                "cabinets_objects",
                "table_object_category",
                "cabinet_categories",
            ],
            output_keys=[
                "cabinet_objects",
                "cabinets_objects",
                "table_object_category",
                "cabinet_categories",
                "cabinet_num",
                "all_cabinet_open",
            ],
        )

        with check_cabinet_sm:
            for i, pose in enumerate(self.cabinet_pose):
                prefix = f"CABINET_{i}"

                smach.StateMachine.add(
                    f"{prefix}_GO_TO",
                    GoToLocation(pose),
                    transitions={
                        "succeeded": f"{prefix}_SAY_CHECKING",
                        "failed": f"{prefix}_SAY_CHECKING",
                    },
                )

                smach.StateMachine.add(
                    f"{prefix}_SAY_CHECKING",
                    Say(text=f"Checking cabinet {i}"),
                    transitions={
                        "succeeded": f"{prefix}_WAIT_OPEN",
                        "aborted": f"{prefix}_WAIT_OPEN",
                        "preempted": f"{prefix}_WAIT_OPEN",
                    },
                )

                smach.StateMachine.add(
                    f"{prefix}_WAIT_OPEN",
                    CheckDoorStatus(
                        expected_closed_depth=0.3, change_thresh=0.4, open_thresh=0.6
                    ),
                    transitions={
                        "open": f"{prefix}_SAY_OPEN",
                        "closed": f"{prefix}_SAY_CLOSED",
                        "error": f"{prefix}_SAY_CLOSED",
                    },
                )

                smach.StateMachine.add(
                    f"{prefix}_SAY_OPEN",
                    Say(text="Cabinet is open"),
                    transitions={
                        "succeeded": f"{prefix}_SAY_FAKE_DETECT",
                        "aborted": f"{prefix}_SAY_FAKE_DETECT",
                        "preempted": f"{prefix}_SAY_FAKE_DETECT",
                    },
                )

                smach.StateMachine.add(
                    f"{prefix}_SAY_CLOSED",
                    Say(text="Cabinet is closed"),
                    transitions={
                        "succeeded": f"{prefix}_OPEN_DOOR",
                        "aborted": f"{prefix}_OPEN_DOOR",
                        "preempted": f"{prefix}_OPEN_DOOR",
                    },
                )

                smach.StateMachine.add(
                    f"{prefix}_OPEN_DOOR",
                    Say(text="Opening the cabinet is ongoing."),
                    transitions={
                        "succeeded": f"{prefix}_SAY_FAKE_DETECT",
                        "aborted": f"{prefix}_SAY_FAKE_DETECT",
                        "preempted": f"{prefix}_SAY_FAKE_DETECT",
                    },
                )

                smach.StateMachine.add(
                    f"{prefix}_SAY_FAKE_DETECT",
                    Say(
                        text="Yolo is not trained for cabinet. I will give a fake cabinet objects for LLM."
                    ),
                    transitions={
                        "succeeded": f"{prefix}_CLASSIFY",
                        "aborted": f"{prefix}_CLASSIFY",
                        "preempted": f"{prefix}_CLASSIFY",
                    },
                )

                # smach.StateMachine.add(
                #     f"{prefix}_DETECT",
                #     DetectDict(),
                #     transitions={
                #         "succeeded": f"{prefix}_CLASSIFY",
                #         "failed": f"{prefix}_CLASSIFY",
                #     },
                #     remapping = {'detections': 'cabinet_objects'}
                # )

                smach.StateMachine.add(
                    f"{prefix}_APPEND",
                    AppendDetections(),
                    transitions={"succeeded": f"{prefix}_CLASSIFY"},
                    remapping={
                        "cabinet_objects": f"cabinet_objects",
                        "cabinets_objects": f"cabinets_objects",
                    },
                )

                smach.StateMachine.add(
                    f"{prefix}_CLASSIFY",
                    ClassifyCategory("cabinet"),
                    transitions={
                        "succeeded": f"{prefix}_NEXT",
                        "failed": f"{prefix}_NEXT",
                        "empty": f"{prefix}_NEXT",
                    },
                )

                if i < len(self.cabinet_pose) - 1:
                    smach.StateMachine.add(
                        f"{prefix}_NEXT",
                        CBState(
                            lambda userdata: "continue",
                            outcomes=["continue"],
                            input_keys=["all_cabinet_open"],
                            output_keys=["all_cabinet_open"],
                        ),
                        transitions={"continue": f"CABINET_{i+1}_GO_TO"},
                    )
                else:
                    smach.StateMachine.add(
                        f"{prefix}_NEXT",
                        CBState(
                            lambda userdata: "done",
                            outcomes=["done"],
                            input_keys=["all_cabinet_open"],
                            output_keys=["all_cabinet_open"],
                        ),
                        transitions={"done": "done"},
                    )

        smach.StateMachine.add(
            "CHECK_CABINET",
            check_cabinet_sm,
            transitions={"done": "SET_CABINET_CHECKED_TRUE"},
            remapping={
                "table_object": "table_object",
                "cabinets_objects": "cabinets_objects",
                "table_object_category": "table_object_category",
                "cabinet_categories": "cabinet_categories",
                "cabinet_num": "cabinet_num",
                "all_cabinet_open": "all_cabinet_open",
            },
        )

        smach.StateMachine.add(
            "SET_CABINET_CHECKED_TRUE",
            CBState(
                set_cabinet_checked_cb,
                outcomes=["done"],
                input_keys=["all_cabinet_open"],
                output_keys=["all_cabinet_open"],
            ),
            transitions={"done": "SAY_CABINET_CATEGORY"},
            remapping={"all_cabinet_open": "all_cabinet_open"},
        )
