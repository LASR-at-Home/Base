import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import smach
from geometry_msgs.msg import Pose, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from std_msgs.msg import Header
from lasr_skills import GoToLocation, Listen, GetNameAndDrink
from receptionist.states.print_state import SayTemp
# from shapely.geometry import Polygon




class Receptionist(smach.StateMachine, Node):
    def __init__(
            self, 
            wait_pose : Pose, 
            # wait_area : Polygon,
            seat_pose : Pose, 
            # seat_area : Polygon,
            host_data

    ):
        Node.__init__(self,"test_state_machine")
        smach.StateMachine.__init__(
            self, 
            outcomes=["succeeded","aborted","timed_out","failed"],
            
        )

        self.wait_pose = wait_pose
        # self.wait_area = wait_area
        self.seat_pose = seat_pose
        # self.seat_area = seat_area

        with self: 
            self.userdata.guest_data = {
                "host": host_data,
                "guest1": {"name": "", "drink": "", "detection": False},
                "guest2": {"name": "", "drink": "", "detection": False},
            }

            smach.StateMachine.add(
                "SAY_START",
                SayTemp(text="Start of receptionist task."),
                transitions={
                    "succeeded": "GO_TO_WAIT_LOCATION_GUEST_1",
                    "aborted": "GO_TO_WAIT_LOCATION_GUEST_1",
                    "preempted": "GO_TO_WAIT_LOCATION_GUEST_1",
                },
            )

            self._goto_waiting_area(guest_id=1)

            smach.StateMachine.add(
                "GET_NAME_AND_DRINK_1",
                GetNameAndDrink(text="Hi, what is your name and favourite drink"),
                transitions={
                    "succeeded": "GO_TO_WAIT_LOCATION_GUEST_1",
                    "aborted": "GO_TO_WAIT_LOCATION_GUEST_1",
                    "preempted": "GO_TO_WAIT_LOCATION_GUEST_1",
                },
            )


            


            #next, we want to deal with the guest. So we need to ask: 



            smach.StateMachine.add(
                "GO_TO_SEATING_AREA",
                GoToLocation(location=seat_pose),
                transitions={"succeeded":"GO_TO_WAIT_LOCATION_GUEST_2","failed":"aborted"}

            )

            self._goto_waiting_area(guest_id=2)




    def _goto_waiting_area(self, guest_id:int) -> None: 
        """Adds the states to go to the waiting area.

        Args:
            guest_id (int): Identifier for the guest.
        """
            
        smach.StateMachine.add(
                f"GO_TO_WAIT_LOCATION_GUEST_{guest_id}",
                GoToLocation(location=self.wait_pose),
                transitions={"succeeded": f"SAY_WAITING_GUEST_{guest_id}", "failed":"aborted"},
            )
        
        smach.StateMachine.add(
            f"SAY_WAITING_GUEST_{guest_id}",
            SayTemp(text="I am waiting for a guest."),
            transitions={
                "succeeded": "ASK_NAME_AND_DRINK_1",
                "aborted":  "ASK_NAME_AND_DRINK_1",
                "preempted":  "ASK_NAME_AND_DRINK_1",
            },
        )

        





