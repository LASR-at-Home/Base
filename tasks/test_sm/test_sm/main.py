from typing import Union
import rclpy
import smach
from geometry_msgs.msg import Pose, PoseStamped, Point, Polygon, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator
from std_msgs.msg import Header
from test_sm.state_machine import TestStateMachine
import sys




def get_pose_from_param(param_name,node):
    #param_name should be a string - top of the dictionary 




    target_pose = Pose(
        position=Point(
           x = node.get_parameter(param_name+'.position.x').get_parameter_value()._double_value,
           y = node.get_parameter(param_name+'.position.y').get_parameter_value()._double_value,
           z = node.get_parameter(param_name+'.position.z').get_parameter_value()._double_value,            
        ),
        orientation=Quaternion(
            x = node.get_parameter(param_name+'.orientation.x').get_parameter_value()._double_value,
            y = node.get_parameter(param_name+'.orientation.y').get_parameter_value()._double_value,
            z = node.get_parameter(param_name+'.orientation.z').get_parameter_value()._double_value,
            w = node.get_parameter(param_name+'.orientation.w').get_parameter_value()._double_value

        )
        
    )

    return target_pose


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('main_sm',allow_undeclared_parameters=True,automatically_declare_parameters_from_overrides=True)
    # node = rclpy.create_node('main_sm',allow_undeclared_parameters=True)
    node.declare_parameter("start_pose")
    node.declare_parameter("end_pose")
    node.get_logger().info('Created node')




    
    # start_pose = node.get_parameter('start_pose.orientation.w').get_parameter_value()._double_value
    # end_pose = node.get_parameter('end_pose').get_parameter_value()
    start_pose = get_pose_from_param('start_pose',node)
    end_pose = get_pose_from_param('end_pose',node)


    # start_pose = Pose()
    # end_p
    node.get_logger().info(f'Start Pose: {start_pose}')
    node.get_logger().info(f'End pose: {end_pose}')

    receptionist = TestStateMachine(
        start_pose,
        end_pose,
    )

    outcome = receptionist.execute()


    rclpy.spin(node)
if __name__ == "__main__":
    main()


