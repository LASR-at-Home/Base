from typing import Union
import rclpy
import smach
from geometry_msgs.msg import Pose, PoseStamped, Point, Polygon, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator
from std_msgs.msg import Header
from receptionist.state_machine import Receptionist
import sys
# from shapely.geometry import Polygon



def get_polygon_from_param(param_name,node):
    param_array = node.get_parameter(param_name).get_parameter_value()._array_value

    return Polygon(param_array)


def get_pose_from_param(param_name, node):
    # param_name should be a string - top of the dictionary

    target_pose = Pose(
        position=Point(
            x=node.get_parameter(param_name + ".position.x")
            .get_parameter_value()
            ._double_value,
            y=node.get_parameter(param_name + ".position.y")
            .get_parameter_value()
            ._double_value,
            z=node.get_parameter(param_name + ".position.z")
            .get_parameter_value()
            ._double_value,
        ),
        orientation=Quaternion(
            x=node.get_parameter(param_name + ".orientation.x")
            .get_parameter_value()
            ._double_value,
            y=node.get_parameter(param_name + ".orientation.y")
            .get_parameter_value()
            ._double_value,
            z=node.get_parameter(param_name + ".orientation.z")
            .get_parameter_value()
            ._double_value,
            w=node.get_parameter(param_name + ".orientation.w")
            .get_parameter_value()
            ._double_value,
        ),
    )

    return target_pose


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node(
        "receptionist_sm",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )
    node.declare_parameter("wait_pose")
    node.declare_parameter("seat_pose")
    node.get_logger().info("Created node")

    wait_pose = get_pose_from_param("wait_pose", node)
    # wait_area = get_polygon_from_param("wait_area",node)
    seat_pose = get_pose_from_param("seat_pose", node)
    # seat_area = get_polygon_from_param("seat_area",node)
    # sofa_area = get_polygon_from_param("sofa_area",node)
    


    node.get_logger().info(f"Start Pose: {wait_pose}")
    node.get_logger().info(f"End pose: {seat_pose}")



    receptionist = Receptionist(
        wait_pose,
        # wait_area,
        seat_pose,
        # seat_area,
        {
            "name": "john",
            "drink": "milk",
            "dataset": "receptionist",
            "detection": False,
        },
    )

    outcome = receptionist.execute()

    rclpy.spin(node)


if __name__ == "__main__":
    main()
