### SMACH on ROS2

SMACH is not officially supported in ROS Humble. The ros2 branch of `ros/executive_smach` provides a compatible wrapper providing supported version of SMACH. 

#### Key Notes

##### State Machines
- State Machines remain mainly unchanged from ROS Noetic. 

##### States
- There exists 2 usable version of states. 
    1. `smach.State`
        - This is the base version build upon the smach library. 
        - It can be used for non-ROS applications as it is not dependent on it. 
        - from `executive_ros` package:
            ```python
                class State(object):
                """Base class for SMACH states.

                A SMACH state interacts with SMACH containers in two ways. The first is its
                outcome identifier, and the second is the set of userdata variables which
                it reads from and writes to at runtime. Both of these interactions are
                declared before the state goes active (when its C{execute()} method is
                called) and are checked during construction.
                """
                def __init__(self, outcomes=[], input_keys=[], output_keys=[], io_keys=[]):
                    ...
            ```

    2. `RosState` (**Use This for ROS2 dependent packages**)
        - This is a subclass of `smach.State` which is designed for use in ROS-based applications. 
             - For instance, this state has contains a parameter which allows a `node` to be passed. while `smach.State` does not.
        - from `executive_ros` package:
            ```python
                class RosState(smach.State):
                """
                A state that can interact with a ROS node.
                """
                def __init__(self, node, **kwargs):
                    ...
            ```

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

#### Reference
- For example code, refer to `skills/src/lasr_skills/` folder 
- Modules and Classes provided by [executive_smach](https://github.com/ros/executive_smach) can be found on thier Github page.