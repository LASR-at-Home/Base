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
from lasr_skills import GoToLocation, Say, Wait, PlayMotion
from restaurant.states import FaceCustomer
from restaurant.states.build_phrases import BuildPlaceOrderPhrase, BuildAnnounceOrderPhrase
from rclpy.node import Node


class GetOrderFromBar(yasmin.StateMachine):

    def __init__(self):
        super().__init__(outcomes=["succeeded", "failed"])
        self.add_input_key("order")
        self.add_input_key("bar_pose")

        self.node = yasmin_ros.logger_node

        parameters = {
            "barman": self.get_pose("barman_pose"),
            "wait_duration": self.get_value("wait_duration"),
            "ordered_food": self.get_value("ordered_food"),
        }

        yasmin.YASMIN_LOG_INFO(f"parameters {parameters}")

        # self.add_state(
        #     "GO_TO_BAR",
        #     GoToLocation(),
        #     transitions={"succeeded": "FACE_BARMAN", "failed": "failed"},
        #     remappings={"location": "bar_pose"},
        # )

        self.add_state(
            "FACE_BARMAN",
            GoToLocation(location=parameters["barman"]),
            transitions={"succeeded": "PLACE_ORDER", "failed": "failed"},
        )
        
        self.add_state(
            "BUILD_PLACE_ORDER",
            BuildPlaceOrderPhrase(),
            transitions={"succeeded": "PLACE_ORDER", "failed": "failed"},
        )

        self.add_state(
            "PLACE_ORDER",
            Say(),
            transitions={
                "succeeded": "WAIT_FOR_ORDER",
                "aborted": "failed",
                "canceled": "failed",
            },
            remappings={"text": "place_order_phrase"},
        )

        self.add_state(
            "WAIT_FOR_ORDER",
            Wait(wait_time=parameters["wait_duration"]),
            transitions={"succeeded": "GO_TO_TABLE", "failed": "failed"},
        )

        self.add_state(
            "GO_TO_TABLE",
            GoToLocation(),
            transitions={"succeeded": "FACE_CUSTOMER", "failed": "failed"},
            remappings={"location": "location"},
        )

        self.add_state(
            "FACE_CUSTOMER",
            FaceCustomer(),
            transitions={"succeeded": "LOOK_AT_CUSTOMER", "failed": "LOOK_AT_CUSTOMER"},
        )

        self.add_state(
            "LOOK_AT_CUSTOMER",
            PlayMotion(motion_name="look_centre"),
            transitions={
                "succeeded": "BUILD_ANNOUNCE_ORDER_PHRASE",
                "aborted": "BUILD_ANNOUNCE_ORDER_PHRASE",
                "canceled": "BUILD_ANNOUNCE_ORDER_PHRASE",
            },
        )

        self.add_state(
            "BUILD_ANNOUNCE_ORDER_PHRASE",
            BuildAnnounceOrderPhrase(),
            transitions={"succeeded": "ANNOUNCE_ORDER", "failed": "failed"},
        )

        self.add_state(
            "ANNOUNCE_ORDER",
            Say(),
            transitions={
                "succeeded": "succeeded",
                "aborted": "failed",
                "canceled": "failed",
            },
            remappings={"text": "announce_order_phrase"},
        )

    def get_pose(self, pose_key):
        pose = Pose(
            position=Point(
                x=float(self.get_value(f"{pose_key}.position.x")),
                y=float(self.get_value(f"{pose_key}.position.y")),
                z=float(self.get_value(f"{pose_key}.position.z")),
            ),
            orientation=Quaternion(
                x=float(self.get_value(f"{pose_key}.orientation.x")),
                y=float(self.get_value(f"{pose_key}.orientation.y")),
                z=float(self.get_value(f"{pose_key}.orientation.z")),
                w=float(self.get_value(f"{pose_key}.orientation.w")),
            ),
        )
        return pose

    def get_value(self, key):
        return self.node.get_parameter(key).value


try:
    from rclpy.executors import EventsExecutor as Executor
except ImportError:
    from rclpy.executors import MultiThreadedExecutor as Executor
from threading import Thread


class RestaurantNode(Node):
    def __init__(self):
        super().__init__(
            node_name="restaurant",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self._executor = Executor()
        self._executor.add_node(self)
        self._spin_thread = Thread(target=self._executor.spin)
        self._spin_thread.start()


def main():

    rclpy.init()
    node = RestaurantNode()

    yasmin_ros.set_ros_loggers(node)

    sm = GetOrderFromBar()

    try:
        bb = yasmin.Blackboard()
        outcome = sm(bb)
        yasmin.YASMIN_LOG_INFO(f"GetOrderFromBar finished with outcome {outcome}")

    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
