# GUIDE TO PORTING AND RUNNING YASMIN

**YASMIN docs can be found [here](https://github.com/uleroboticsgroup/yasmin/tree/main)**

## PORTING

### STATES

States that are to be ported from SMACH to YASMIN can be ported easily, as much of the functionality is the same. However nodes are treated differently, and userdata (now referred to as blackboard) is drastically different.

Blackboard now replaces userdata and is a **global hashmap (dictionary)** shared across all YASMIN state machines and states.

For all states, replace all imports of:
```
import smach
import smach_ros
```
to:
```
import yasmin
import yasmin_ros # Only if using special states like service 
```

Now go ahead and for the state, replace:
```
class MyState(smach_ros.RosState):
    def __init__(self, node, args here):
        super().__init__(node, outcomes, input_keys, output_keys)

    def execute(self, userdata):
        userdata.key = value
```
to:
```
class MyState(yasmin.State):
    def __init__(self, args here):
        super().__init__(outcomes)
        self.add_input_key(input) # DO THIS FOR ALL INPUTS IN input_keys
        self.add_output_key(output) # DO THIS FOR ALL OUTPUTS IN output_keys
        self.node = yasmin_ros.logger_node # or assign to self._node, depending on how the node is called in the state

    def execute(self, blackboard):
        blackboard['key'] = value
```

For any logs, please replace (view this [file](https://github.com/uleroboticsgroup/yasmin/blob/main/yasmin/yasmin/__init__.py) to see more information):
```
self.node.get_logger().info('information')
```
to:
```
yasmin.YASMIN_LOG_INFO('information') # can also be WARN, ERROR, DEBUG
```

#### SPECIAL STATES (ACTION, SERVICE, PUBLISHER, ETC.)

For changing a state to the YASMIN version for an action client, service client, publisher, etc. please review the YASMIN docs linked above [here](#guide-to-porting-and-using-yasmin)

### STATE MACHINES

When porting state machines, you must change:
```
class MyStateMachine(smach.StateMachine):
    def __init__(self, node, args here):
        super().__init__(outcomes=[], input_keys=[], output_keys=[])
        
        self.userdata.key = value

        self.add('STATE', MyState(node=node, args here), transtions={})
```
to:
```
class MyStateMachine(yasmin.StateMachine):
    def __init__(self, args here):
        super().__init__(outcomes=[], handle_sigint=True)
        self.add_input_key(input) # DO THIS FOR ALL INPUTS IN input_keys
        self.add_output_key(output) # DO THIS FOR ALL OUTPUTS IN output_keys
        
        # Userdata changed to blackboard, and for state machiens blackboard handled outside of construction of object

        self.add_state('STATE', MyState(args here), transitions={}) # If before there was a remapping parameter, change to remappings
```

## RUNNING

When running the state machine, your main() must look similar to this:
```
def main():
    rclpy.init()

    yasmin_ros.set_ros_loggers() # You can pass a node as an argument in order to keep your own name, for now leave like this, as this is still being investigated
    
    sm = MyStateMachine() 
    # or if just for a state do:
    # sm = yasmin.StateMachine(outcomes=[], handle_sigint=True)
    # sm.add_state(...)

    # optionally, if you wanted to view the state machine do:
    # from yasmin_viewer import YasminViewPub
    # YasminViewPub(sm)

    try:
        outcome=sm()
        yasmin.YASMIN_LOG_INFO(f"State machine finished with outcome {outcome}")

        # optionally, you can add in a filled in blackboard like so:
        # from yasmin import BlackBoard
        # blackboard = BlackBoard()
        # bb['key'] = value
        # outcome = sm(bb)
        # yasmin.YASMIN_LOG_INFO(f"State machine finished with outcome {outcome}")

    except Exception as e:
        yasmin.YASMIN_LOG_WARN(e)

    if rclpy.ok():
        rclpy.shutdown()
```

