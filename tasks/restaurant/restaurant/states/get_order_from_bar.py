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
import yasmin_ros
from geometry_msgs.msg import Pose
from lasr_skills import GoToLocation, Say, Wait, FacePerson


def dict_to_pose(data: dict) -> Pose:
    """Convert a dictionary with position and orientation to a Pose message."""
    pose = Pose()
    if "position" in data:
        pose.position.x = data["position"]["x"]
        pose.position.y = data["position"]["y"]
        pose.position.z = data["position"]["z"]
    if "orientation" in data:
        pose.orientation.x = data["orientation"]["x"]
        pose.orientation.y = data["orientation"]["y"]
        pose.orientation.z = data["orientation"]["z"]
        pose.orientation.w = data["orientation"]["w"]
    return pose


class GetOrderFromBar(yasmin.StateMachine):

    def __init__(self, node):
        super().__init__(
            outcomes=["succeeded", "failed"], handle_sigint=True
        )
        self.node = node
        
        bar_location_dict = node.get_parameter("get_order_from_bar.bar_location").value
        barman_location_dict = node.get_parameter("get_order_from_bar.barman_location").value
        table_location_dict = node.get_parameter("get_order_from_bar.table_location").value
        ordered_food = node.get_parameter("get_order_from_bar.ordered_food").value
        guest_location_dict = node.get_parameter("get_order_from_bar.guest_location").value

        wait_duration = node.get_parameter("get_order_from_bar.wait_duration").value
        
        # Convert dictionaries to Pose objects
        bar_location = dict_to_pose(bar_location_dict)
        barman_location = dict_to_pose(barman_location_dict)
        table_location = dict_to_pose(table_location_dict)
        guest_location = dict_to_pose(guest_location_dict)

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
            Wait(duration=wait_duration),
            transitions={"succeeded": "succeeded", "failed": "failed"}
        )
        
        self.add_state(
            "GO_TO_TABLE",
            GoToLocation(location=table_location),
            transitions={"succeeded": "ANNOUNCE_ORDER", "failed": "failed"}
        )
        
        self.add_state(
            "FACE_GUESTS",
            GoToLocation(location=guest_location),
            transitions={"succeeded": "ANNOUNCE_ORDER", "failed": "failed"}
        )
        
        self.add_state(
            "ANNOUNCE_ORDER",
            Say(text=f"Your order of {', '.join(ordered_food)} is ready!"),
            transitions={"succeeded": "succeeded", "failed": "failed"}
        )
    
def main(args=None):
    rclpy.init(args=args)
    
    node = rclpy.create_node(
        node_name="get_order_from_bar",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )
    
    sm = GetOrderFromBar(node=node)
    yasmin_ros.set_ros_loggers(node)

    try:        
        bb = yasmin.Blackboard()
        outcome = sm(bb)
        yasmin.YASMIN_LOG_INFO(f"GetOrderFromBar finished with outcome {outcome}")

    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == "__main__":
    main()
