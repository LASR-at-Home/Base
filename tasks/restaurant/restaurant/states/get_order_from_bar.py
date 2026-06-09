"""
Going to bar
Face barman
Say order to Barman
Wait for order to be placed on tray (time can be constant, e.g. 30 sec)
Navigate back to table
Face guests
Announce order
"""

import rclpy
import yasmin
from geometry_msgs.msg import Pose
from lasr_skills import GoToLocation, Say, Wait, FacePerson

class GetOrderFromBar(StateMachine):

    def __init__(
        self,
        bar_location: Pose,
        barman_location: Pose,
        ordered_food: list[str],
        table_location: Pose
    ):
        super().__init__(
            outcomes=["succeeded", "failed"],
                         handle_sigint=True
                         )
        self.add_state(
            "GO_TO_BAR",
            GoToLocation(location=bar_location),
            transitions={"succeeded": "FACE_BARMAN", "failed": "failed"}
        )
        
        self.add_state(
            "FACE_BARMAN",
            GoToLocation(location=barman_location),
            transitions={"succeeded": "PLACE_ORDER", "failed": "failed"}
        )
        self.add_state(
            "PLACE_ORDER",
            Say(text=f"I would like to order {', '.join(ordered_food)}"),
            transitions={"succeeded": "WAIT_FOR_ORDER", "failed": "failed"}
        )
        
        self.add_state(
            "WAIT_FOR_ORDER",
            Wait(duration=30),  # Assuming a constant wait time of 30 seconds
            transitions={"succeeded": "succeeded", "failed": "failed"}
        )
        
        self.add_state(
            "GO_TO_TABLE",
            GoToLocation(location=table_location),
            transitions={"succeeded": "ANNOUNCE_ORDER", "failed": "failed"}
        )
        
        self.add_state(
            "FACE_GUESTS",
            FacePerson(node=self.node),
            transitions={"succeeded": "ANNOUNCE_ORDER", "failed": "failed"}
        )
        
        self.add_state(
            "ANNOUNCE_ORDER",
            Say(text=f"Your order of {', '.join(ordered_food)} is ready!"),
            transitions={"succeeded": "succeeded", "failed": "failed"}
        )
    
def main():
    rclpy.init()
    
    sm = GetOrderFromBar(
        bar_location=Pose(),
        barman_location=Pose(),
        ordered_food=["pizza", "soda"],
        table_location=Pose()
    )

    try:
        outcome=sm()
        yasmin.YASMIN_LOG_INFO(f"GetOrderFromBar finished with outcome {outcome}")

    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)

    if rclpy.ok():
        rclpy.shutdown()
