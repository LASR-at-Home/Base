import smach

from lasr_skills import Say, AdjustCamera, GoToLocation, CheckDoorStatus
from storing_groceries.states import (
    SelectObject, ClassifyCategory, SayDynamic
)

class ObjectSortingLoop(smach.StateMachine):
    def __init__(self, table_pose, cabinet_pose):        
        super().__init__(
            outcomes=["succeeded","failed","escape"],
            input_keys=[],
        )

        self.table_pose = table_pose
        self.cabinet_pose = cabinet_pose

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
        self.all_cabinet_open = False


        with self:
            self.go_to_table()

            smach.StateMachine.add(
                "DETECT_TABLE",
                Say(text="Detect table is on going. Will continue with fake data"),
                transitions={
                    "succeeded": "SELECT_OBJECT",
                    "aborted": "SELECT_OBJECT",
                    "preempted": "SELECT_OBJECT",
                },
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
                    "succeeded": "GO_TO_CABINET",
                    "aborted": "GO_TO_CABINET",
                    "preempted": "GO_TO_CABINET",
                },
            )

            self.go_to_cabinet() 

            self.check_cabinet_open() #try do once

            smach.StateMachine.add(
                "DETECT_CABINET",
                Say(text="DETECT_CABINET is on going"),
                transitions={
                    "succeeded": "CLASSIFY_CATEGORY_CABINET",
                    "aborted": "CLASSIFY_CATEGORY_CABINET",
                    "preempted": "CLASSIFY_CATEGORY_CABINET",
                },
            )

            smach.StateMachine.add(
                "CLASSIFY_CATEGORY_CABINET", #try do once (for new collect at once and edit whenever storing new category to follow object category)
                ClassifyCategory("cabinet"),
                transitions={
                    "succeeded": "SAY_CABINET_CATEGORY",
                    "failed": "SAY_CABINET_CATEGORY",
                    "empty": "SAY_CABINET_CATEGORY",
                },
            )

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
                SayDynamic(text_fn=lambda ud: f"{ud.table_object['name']}'s belongs to {ud.cabinet_num}"),                
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
                    "succeeded": "GO_TO_TABLE",
                    "aborted": "GO_TO_TABLE",
                    "preempted": "GO_TO_TABLE",
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
                "succeeded": f"WAIT_CABINET_OPEN",
                "aborted": f"WAIT_CABINET_OPEN",
                "preempted": f"WAIT_CABINET_OPEN",
            },
        )

    def check_cabinet_open(self) -> None:
        """Adds the states to open the cabinet.
        """
        #Todo: door opening manipulation. But also checking every shelf. points.

        smach.StateMachine.add(
            "WAIT_CABINET_OPEN",
            CheckDoorStatus(
                expected_closed_depth=1.2,  # adjust for cabinet (~0.5) or room door (~1.2)
                change_thresh=0.4,
                open_thresh=0.6),
            transitions={
                "open": "SAY_SHELF_OPEN", 
                "closed": "SAY_SHELF_CLOSED",
                "error": "SAY_SHELF_CLOSED",
            }
        )

        #check all shelf level ongoing
        
        smach.StateMachine.add(
            f"SAY_SHELF_OPEN",
            Say(text="Cabinet is open"),
            transitions={
                "succeeded": f"DETECT_CABINET",
                "aborted": f"DETECT_CABINET",
                "preempted": f"DETECT_CABINET",
            },
        )

        smach.StateMachine.add(
            f"SAY_SHELF_CLOSED",
            Say(text="Cabinet is closed"),
            transitions={
                "succeeded": f"CABINET_DOOR_OPEN",
                "aborted": f"CABINET_DOOR_OPEN", 
                "preempted": f"CABINET_DOOR_OPEN",
            },
        )

        smach.StateMachine.add(
            f"CABINET_DOOR_OPEN",
            Say(text="Openning the cabinet is ongoing."),
            transitions={
                "succeeded": f"DETECT_CABINET",
                "aborted": f"DETECT_CABINET", 
                "preempted": f"DETECT_CABINET",
            },
        )
