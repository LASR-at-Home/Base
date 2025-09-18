import rospy
import smach
import smach_ros
from .states import Start, GoToTable, CheckTable

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion


class Phase1(smach.StateMachine):
    class GoIdle(smach.StateMachine):
        def __init__(self):
            smach.StateMachine.__init__(self, outcomes=["done"])

            idle_location = rospy.get_param("/coffee_shop/counter/location")
            
            idle_position, idle_orientation = (
                idle_location["position"],
                idle_location["orientation"],
            )
            idle_pose = Pose(
                position=Point(**idle_position),
                orientation=Quaternion(**idle_orientation),
            )
            idle_goal = MoveBaseGoal()
            idle_goal.target_pose.header.frame_id = "map"
            idle_goal.target_pose.pose = idle_pose

            with self:

                smach.StateMachine.add(
                    "GO_TO_IDLE",
                    smach_ros.SimpleActionState(
                        "move_base", MoveBaseAction, goal=idle_goal
                    ),
                    transitions={
                        "succeeded": "IDLE",
                        "aborted": "IDLE",
                        "preempted": "IDLE",
                    },
                )

                @smach.cb_interface(
                    input_keys=[], output_keys=[], outcomes=["succeeded"]
                )
                def idle_cb(_):
                    rospy.sleep(rospy.Duration(15.0))
                    return "succeeded"

                smach.StateMachine.add(
                    "IDLE",
                    smach.CBState(idle_cb),
                    transitions={"succeeded": "done"},
                )

    class Start(smach.State):
        def __init__(self, context):
            smach.State.__init__(self, outcomes=["done", "not_done"])
            self.context = context

        def execute(self, userdata):
            # TODO maybe improve this logic
            self.context.say("Please say 'start' when you need my help.")
            for _ in range(10):
                rospy.sleep(1.0)
                #utterance = self.context.listen()
                utterance = "start"
                if utterance and "start" in utterance.lower():
                    self.context.say("I will start my work now.")
                    return "done"
            return "not_done"

    def __init__(self, context):
        smach.StateMachine.__init__(self, outcomes=["greet_new_customer", "serve"])

        with self:

            @smach.cb_interface(input_keys=[], output_keys=[], outcomes=["done"])
            def reset_tables(ud):
                for table in context.tables.keys():
                    context.tables[table]["status"] = "unvisited"
                return "done"

            smach.StateMachine.add(
                "GO_IDLE", self.GoIdle(), transitions={"done": "START"}
            )
            smach.StateMachine.add(
                "START", self.Start(context), transitions={"done": "RESET_TABLES", "not_done": "START"}
            )

            smach.StateMachine.add(
                "RESET_TABLES",
                smach.CBState(reset_tables),
                transitions={"done": "GO_TO_TABLE"},
            )

            smach.StateMachine.add(
                "GO_TO_TABLE", GoToTable(context), transitions={"done": "CHECK_TABLE"}
            )
            smach.StateMachine.add(
                "CHECK_TABLE",
                CheckTable(context),
                transitions={
                    "not_finished": "GO_TO_TABLE",
                    "has_free_tables": "greet_new_customer",
                    "has_needs_serving_tables": "serve",
                    "idle": "GO_IDLE",
                },
            )
