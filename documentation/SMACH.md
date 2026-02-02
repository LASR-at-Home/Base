### SMACH on ROS2

SMACH is not officially supported in ROS Humble. The ros2 branch of `ros/executive_smach` provides a compatible wrapper proving a 'ros2 aware extension' of smach.State. 

#### Key Notes
State Machines remain mainly unchanged from ROS Noetic. 

- States need a `rclpy.Node` to be passed as parameter during creation.
    ```python
        from smach import StateMachine
        from smach_ros import RosState

        class StateA(RosState):
            def __init__(self, node, args):
                super().__init__(node, args)
                
            def execute(self, userdata):
                pass
        
        class StateMachineExample(StateMachine):
            def __init__(self, node):
                super().__init__(node, args)

                with self:
                    StateMachine.add('StateA', StateA(node), transition={})

        def main():
            rclpy.init()
            node = rclpy.create_node("state_machine_example")
            sm = StateMachineExample(node)

            node.get_logger().info("Starting the state machine...")
            outcome = sm.execute()

            rclpy.shutdown()
        
        if __name__ == '__main__':
            main()

    ```
- For preemption differences on ROS2, see [PREEMPTION.md](PREEMPTION.md).

#### Reference
- For example code, refer to `skills/src/lasr_skills/` folder 
- Modules and Classes provided by [executive_smach](https://github.com/ros/executive_smach) can be found on thier Github page.