import smach
from smach import CBState
from lasr_skills import Say, AdjustCamera, GoToLocation, CheckDoorStatus, DetectDict
from storing_groceries.states import (
    SelectObject, ClassifyCategory, SayDynamic
)
def is_cabinet_checked_cb(userdata):
    return "true" if userdata.all_cabinet_open else "false"

def set_cabinet_checked_cb(userdata):
    userdata.all_cabinet_open = True
    return "done"


class ObjectSortingLoop(smach.StateMachine):
    def __init__(self, table_pose, cabinet_pose):        
        super().__init__(
            outcomes=["succeeded","failed","escape"],
            input_keys=[],
            output_keys=[],
        )

        self.table_pose = table_pose
        self.cabinet_pose = [cabinet_pose]

        self.userdata.table_objects = [{"name": "apple", "confidence": 0.8, "bbox": [1.0, 0.5, 0.1]}, {"name": "orange", "confidence": 0.85, "bbox": [1.0, 0.5, 0.1]}, {"name": "hat", "confidence": 0.85, "bbox": [1.0, 0.5, 0.1]}]
        self.userdata.table_object = {"name": None, "confidence": None, "bbox": None}
        self.userdata.not_graspable = []
        self.userdata.table_object_category = None
        self.userdata.cabinets_objects = [[{"name": "apple", "confidence": 0.8, "bbox": [1.0, 0.5, 0.1]}, {"name": "orange", "confidence": 0.85, "bbox": [1.0, 0.5, 0.1]}, {"name": "banana", "conf": 0.85, "bbox": [1.0, 0.5, 0.1]}],
                                         [{"name": "hat", "confidence": 0.8, "bbox": [1.0, 0.5, 0.1]}, {"name": "t-shirt", "confidence": 0.85, "bbox": [1.0, 0.5, 0.1]}, {"name": "shorts", "conf": 0.85, "bbox": [1.0, 0.5, 0.1]}],
                                         [{"name": "cola", "confidence": 0.8, "bbox": [1.0, 0.5, 0.1]}, {"name": "cider", "confidence": 0.85, "bbox": [1.0, 0.5, 0.1]}, {"name": "coffee", "conf": 0.85, "bbox": [1.0, 0.5, 0.1]}],
                                         [{"name": "empty", "confidence": 0.8, "bbox": [1.0, 0.5, 0.1]}]
                                        ]
        self.userdata.cabinet_categories = []
        self.userdata.cabinet_num = None
        self.userdata.all_cabinet_open = False

        with self:
            # self.go_to_table()

            smach.StateMachine.add(
                "DETECT_TABLE",
                DetectDict(),
                transitions={
                    "succeeded": "SELECT_OBJECT",
                    "failed": "SELECT_OBJECT",
                },
                remapping = {'detections': 'table_objects'}
            )

            smach.StateMachine.add(
                "SELECT_OBJECT",
                SelectObject(),
                transitions={
                    "succeeded": "SAY_SELECT_OBJECT",
                    "failed": "SAY_SELECT_OBJECT",
                    "escape": "escape",
                },
            )

            smach.StateMachine.add(
                "SAY_SELECT_OBJECT",
                SayDynamic(text_fn=lambda ud: f"{ud.table_object['name']} is selected."),                
                transitions={
                    "succeeded": "CLASSIFY_CATEGORY_OBJECT",
                    "aborted": "CLASSIFY_CATEGORY_OBJECT",
                    "preempted": "CLASSIFY_CATEGORY_OBJECT",
                },
            )

            smach.StateMachine.add(
                "CLASSIFY_CATEGORY_OBJECT",
                ClassifyCategory("object"),
                transitions={
                    "succeeded": "SAY_OBJECT_CATEGORY",
                    "failed": "SAY_OBJECT_CATEGORY",
                    "empty": "SAY_OBJECT_CATEGORY",
                },
            )

            smach.StateMachine.add(
                "SAY_OBJECT_CATEGORY",
                SayDynamic(text_fn=lambda ud: f"{ud.table_object['name']}'s category is {ud.table_object_category}"),                
                transitions={
                    "succeeded": "GRAB_OBJECT",
                    "aborted": "GRAB_OBJECT",
                    "preempted": "GRAB_OBJECT",
                },
            )

            smach.StateMachine.add(
                "GRAB_OBJECT",
                Say(text="GRAB_OBJECT is on going"),
                transitions={
                    "succeeded": "CHECK_IF_CABINET_ALREADY_OPEN",
                    "aborted": "CHECK_IF_CABINET_ALREADY_OPEN",
                    "preempted": "CHECK_IF_CABINET_ALREADY_OPEN",
                },
            )

            # self.go_to_cabinet() 

            self.check_cabinet() #Do once, but consider the posibility that the door will be closed again.

            smach.StateMachine.add(
                "SAY_CABINET_CATEGORY",
                SayDynamic(text_fn=lambda ud: f"cabinet's categories are {ud.cabinet_categories}"),                
                transitions={
                    "succeeded": "LINK_CATEGORY",
                    "aborted": "LINK_CATEGORY",
                    "preempted": "LINK_CATEGORY",
                },
            )

            smach.StateMachine.add(
                "LINK_CATEGORY", 
                ClassifyCategory("link"),
                transitions={
                    "succeeded": "SAY_LINK_CATEGORY",
                    "failed": "SAY_LINK_CATEGORY",
                    "empty": "SAY_LINK_CATEGORY",
                },
            )

            smach.StateMachine.add(
                "SAY_LINK_CATEGORY",
                SayDynamic(text_fn=lambda ud: f"{ud.table_object['name']}'s belongs to {ud.cabinet_num + 1}"),                
                transitions={
                    "succeeded": "PUT_OBJECT",
                    "aborted": "PUT_OBJECT",
                    "preempted": "PUT_OBJECT",
                },
            )

            smach.StateMachine.add(
                "PUT_OBJECT",
                Say(text="PUT_OBJECT is on going"),
                transitions={
                    "succeeded": "DETECT_TABLE",
                    "aborted": "DETECT_TABLE",
                    "preempted": "DETECT_TABLE",
                },
            )

    def go_to_table(self) -> None:
        """Adds the states to go to table area.
        """
        
        smach.StateMachine.add(
            f"GO_TO_TABLE",
            GoToLocation(self.table_pose),
            transitions={
                "succeeded": f"SAY_ARRIVE_TABLE",
                "failed": f"SAY_ARRIVE_TABLE",
            },
        )
    
        smach.StateMachine.add(
            f"SAY_ARRIVE_TABLE",
            Say(text="Arrive table"),
            transitions={
                "succeeded": f"DETECT_TABLE",
                "aborted": f"DETECT_TABLE",
                "preempted": f"DETECT_TABLE",
            },
        )

    def go_to_cabinet(self) -> None:
        """Adds the states to go to cabinet area.
        """
        smach.StateMachine.add(
                f"GO_TO_CABINET",
                GoToLocation(self.cabinet_pose),
                transitions={
                    "succeeded": f"SAY_ARRIVE_CABINET",
                    "failed": f"SAY_ARRIVE_CABINET",
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

    def check_cabinet(self) -> None:
        """Adds the states to check and classify each cabinet pose once, using CBState for runtime branching."""

        # Decide if cabinet has already been checked
        smach.StateMachine.add(
            "CHECK_IF_CABINET_ALREADY_OPEN",
            CBState(is_cabinet_checked_cb, outcomes=["true", "false"], input_keys=["all_cabinet_open"]),
            transitions={
                "true": "SAY_CABINET_ALREADY_CHECKED",
                "false": "CHECK_CABINET"
            }
        )


        # If already checked, skip to next
        smach.StateMachine.add(
            "SAY_CABINET_ALREADY_CHECKED",
            Say(text="Cabinet already checked"),
            transitions={
                "succeeded": "SAY_CABINET_CATEGORY",
                "aborted": "SAY_CABINET_CATEGORY",
                "preempted": "SAY_CABINET_CATEGORY"
            },
        )

        # Sub-state machine to loop through each cabinet pose
        check_cabinet_sm = smach.StateMachine(outcomes=["done"],
                                            input_keys=["all_cabinet_open", "table_object", "cabinets_objects", "table_object_category", "cabinet_categories"],
                                            output_keys=["table_object_category", "cabinet_categories", "cabinet_num", "all_cabinet_open"],

        )

        with check_cabinet_sm:
            for i, pose in enumerate(self.cabinet_pose):
                prefix = f"CABINET_{i}"

                smach.StateMachine.add(
                    f"{prefix}_SAY_CHECKING",
                    Say(text=f"Checking cabinet {i+1}"),
                    transitions={
                        "succeeded": f"{prefix}_WAIT_OPEN",
                        "aborted": f"{prefix}_WAIT_OPEN",
                        "preempted": f"{prefix}_WAIT_OPEN"
                    }
                )

                smach.StateMachine.add(
                    f"{prefix}_WAIT_OPEN",
                    CheckDoorStatus(expected_closed_depth=1.2, change_thresh=0.4, open_thresh=0.6),
                    transitions={
                        "open": f"{prefix}_SAY_OPEN",
                        "closed": f"{prefix}_SAY_CLOSED",
                        "error": f"{prefix}_SAY_CLOSED"
                    }
                )

                smach.StateMachine.add(
                    f"{prefix}_SAY_OPEN",
                    Say(text="Cabinet is open"),
                    transitions={
                        "succeeded": f"{prefix}_DETECT",
                        "aborted": f"{prefix}_DETECT",
                        "preempted": f"{prefix}_DETECT"
                    }
                )

                smach.StateMachine.add(
                    f"{prefix}_SAY_CLOSED",
                    Say(text="Cabinet is closed"),
                    transitions={
                        "succeeded": f"{prefix}_OPEN_DOOR",
                        "aborted": f"{prefix}_OPEN_DOOR",
                        "preempted": f"{prefix}_OPEN_DOOR"
                    }
                )

                smach.StateMachine.add(
                    f"{prefix}_OPEN_DOOR",
                    Say(text="Opening the cabinet is ongoing."),
                    transitions={
                        "succeeded": f"{prefix}_DETECT",
                        "aborted": f"{prefix}_DETECT",
                        "preempted": f"{prefix}_DETECT"
                    }
                )

                smach.StateMachine.add(
                    f"{prefix}_DETECT",
                    DetectDict(),
                    transitions={
                        "succeeded": f"{prefix}_CLASSIFY",
                        "failed": f"{prefix}_CLASSIFY",
                    },
                    remapping = {'detections': 'cabinets_objects'}
                )

                smach.StateMachine.add(
                    f"{prefix}_CLASSIFY",
                    ClassifyCategory("cabinet"),
                    transitions={
                        "succeeded": f"{prefix}_NEXT",
                        "failed": f"{prefix}_NEXT",
                        "empty": f"{prefix}_NEXT"
                    }
                )

                if i < len(self.cabinet_pose) - 1:
                    smach.StateMachine.add(
                        f"{prefix}_NEXT",
                        CBState(lambda userdata: "continue", outcomes=["continue"], input_keys=["all_cabinet_open"]),
                        transitions={"continue": f"CABINET_{i+1}_SAY_CHECKING"},
                    )
                else:
                    smach.StateMachine.add(
                        f"{prefix}_NEXT",
                        CBState(lambda userdata: "done", outcomes=["done"], input_keys=["all_cabinet_open"]),
                        transitions={"done": "done"}
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
                "all_cabinet_open": "all_cabinet_open"
            }
        )

        smach.StateMachine.add(
            "SET_CABINET_CHECKED_TRUE",
            CBState(set_cabinet_checked_cb, outcomes=["done"]),
            transitions={"done": "SAY_CABINET_CATEGORY"},
            remapping={"all_cabinet_open": "all_cabinet_open"} 
        )
