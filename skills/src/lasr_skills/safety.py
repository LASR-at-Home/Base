import yasmin
import yasmin_ros

from std_msgs.msg import Empty

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Point, PointStamped, Quaternion
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue

from lasr_skills import Say, SafeGoToLocation, DetectDoorOpening

from threading import Thread

try:
    from rclpy.executors import EventsExecutor as Executor
except ImportError:
    from rclpy.executors import MultiThreadedExecutor as Executor

class ChangeNavParam(yasmin_ros.ServiceState):
    def __init__(self):
        super().__init__(
            srv_type=SetParameters,
            srv_name='/global_costmap/global_costmap/set_parameters',
            create_request_handler=self._create_request,
            response_handler=self._handle_resp
        )
        
    def _create_request(self, blackboard):
        request = SetParameters.Request()
        
        value = ParameterValue(double_value=0.70, type=3) # Originally 0.55
        
        param = Parameter(name='inflation_layer.inflation_radius', value=value)
        
        request.parameters = [param]
        
        return request
    
    def _handle_resp(self, blackboard, response):
        return 'succeeded' if response.results[0].successful else 'aborted'
    
class WaitState(yasmin.State):
    def __init__(self):
        super().__init__(outcomes=['succeeded', 'failed'])
        
    def execute(self, blackboard):
        x = str(input("Enter y to proceed with the next point"))
        
        if x == 'y':
            return 'succeeded'
        else:
            return 'failed'
        
class SafetySM(yasmin.StateMachine):
    def __init__(self):
        super().__init__(outcomes=['succeeded', 'failed'])
        
        start_con_sm = yasmin.Concurrence(
            states={
                "SAY_START": Say(text="Start of safety task."),
                "DOOR_START": DetectDoorOpening(),
            },
            default_outcome="failed",
            outcome_map={
                "succeeded": {
                    "SAY_START": "succeeded",
                    "DOOR_START": "door_opened",
                },
            },
        )
        
        def wait_cb(blackboard, msg):
            yasmin.YASMIN_LOG_INFO("RECEIVED START SIGNAL")
            return 'succeeded'
        
        self.add_state(
            'WAIT_START',
            yasmin_ros.MonitorState(
                topic_name='/safety/start',
                outcomes=['succeeded'],
                monitor_handler=wait_cb,
                msg_type=Empty
                ),
            transitions={
                'succeeded': 'START',
                'canceled': 'WAIT_START'
            }
        )
        
        self.add_state(
            'START',
            start_con_sm,
            transitions={
                'succeeded': 'CHANGE_NAV_PARAM', 'failed': 'START'
            }
        )
        
        self.add_state(
            'CHANGE_NAV_PARAM',
            ChangeNavParam(),
            transitions={
                'succeeded': 'SAY_GO_START_POINT',
                'aborted': 'failed'
            }
        )
        
        self.add_state(
            'SAY_GO_START_POINT',
            Say(text='I am now going to the point.'),
            transitions={
                'succeeded': 'GO_TO_START_POINT',
                'aborted': 'failed',
                'canceled': 'failed'
            }
        )
        
        self.add_state(
            'GO_TO_START_POINT',
            SafeGoToLocation(location_param='start_point'),
            transitions={
                'succeeded': 'WAIT_FOR_END_POINT', 'failed': 'GO_TO_START_POINT'
            }
        )
        
        
        
        self.add_state(
            'WAIT_FOR_END_POINT',
            WaitState(),
            transitions={
                'succeeded': 'SAY_GO_END_POINT',
                'failed': 'failed'
            }
        )
        
        self.add_state(
            'SAY_GO_END_POINT',
            Say(text='I am now going to the point.'),
            transitions={
                'succeeded': 'GO_TO_END_POINT',
                'aborted': 'failed',
                'canceled': 'failed'
            }
        )
        
        self.add_state(
            'GO_TO_END_POINT',
            SafeGoToLocation(location_param='end_point'),
            transitions={
                'succeeded': 'succeeded', 'failed': 'GO_TO_END_POINT'
            }
        )
        
class Safety(Node):
    def __init__(self):
        super().__init__(
            node_name="safety",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self._executor = Executor()
        self._executor.add_node(self)
        self._spin_thread = Thread(target=self._executor.spin)
        self._spin_thread.start()
        
def main():
    rclpy.init()
    
    node = Safety()
    yasmin_ros.set_ros_loggers(node)
    
    sm = SafetySM()
    
    sm.set_sigint_handler()
    
    outcome = sm()
    
    yasmin.YASMIN_LOG_INFO(f"State machine has ended with outcome {outcome}")
    

    node.destroy_node()
    rclpy.shutdown()