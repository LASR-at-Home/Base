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
from geometry_msgs.msg import Pose, Point, Quaternion
from lasr_skills import GoToLocation, Say, Wait


def get_pose(key):
    pose = Pose(
        position=Point(
            x=float(
                node.get_parameter(f"{key}.position.x").value
            ),
            y=float(
                node.get_parameter(f"{key}.position.y").value
            ),
            z=float(
                node.get_parameter(f"{key}.position.z").value
            ),
        ),
        orientation=Quaternion(
            x=float(
                node.get_parameter(f"{key}.orientation.x").value
            ),
            y=float(
                node.get_parameter(f"{key}.orientation.y").value
            ),
            z=float(
                node.get_parameter(f"{key}.orientation.z").value
            ),
            w=float(
                node.get_parameter(f"{key}.orientation.w").value
            ),
        ),
    )
    return pose


class GetOrderFromBar(yasmin.StateMachine):

    def __init__(self, node):
        super().__init__(
            outcomes=["succeeded", "failed"], handle_sigint=True
        )
        self.node = node
        
        parameters = {
            "bar": get_pose("get_order_from_bar.bar_location"),
            "barman": get_pose("get_order_from_bar.barman_location"),
            "table": get_pose("get_order_from_bar.table_location"),
            "guest": get_pose("get_order_from_bar.guest_location"),
            "wait_duration": get_pose("get_order_from_bar.wait_duration"),
            "ordered_food": get_pose("get_order_from_bar.ordered_food")
        }

        self.add_state(
            "GO_TO_BAR",
            GoToLocation(location=parameters["bar"]),
            transitions={"succeeded": "FACE_BARMAN", "failed": "failed"}
        )
        
        self.add_state(
            "FACE_BARMAN",
            GoToLocation(location=parameters["barman"]),
            transitions={"succeeded": "PLACE_ORDER", "failed": "failed"}
        )
        self.add_state(
            "PLACE_ORDER",
            Say(text=f"I would like to order {', '.join(parameters['ordered_food'])}"),
            transitions={"succeeded": "WAIT_FOR_ORDER", "failed": "failed"}
        )
        
        self.add_state(
            "WAIT_FOR_ORDER",
            Wait(duration=parameters["wait_duration"]),
            transitions={"succeeded": "succeeded", "failed": "failed"}
        )
        
        self.add_state(
            "GO_TO_TABLE",
            GoToLocation(location=parameters["table"]),
            transitions={"succeeded": "ANNOUNCE_ORDER", "failed": "failed"}
        )
        
        self.add_state(
            "FACE_GUESTS",
            GoToLocation(location=parameters["guest"]),
            transitions={"succeeded": "ANNOUNCE_ORDER", "failed": "failed"}
        )
        
        self.add_state(
            "ANNOUNCE_ORDER",
            Say(text=f"Your order of {', '.join(parameters['ordered_food'])} is ready!"),
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
