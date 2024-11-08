import rospy
import smach
import smach_ros
from geometry_msgs.msg import Point, Pose, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from .states import CheckTable, GoToTable, Start


class Phase1(smach.StateMachine):

    def __init__(self, context):
        smach.StateMachine.__init__(self, outcomes=["serve"])

        with self:

            @smach.cb_interface(input_keys=[], output_keys=[], outcomes=["done"])
            def reset_tables(ud):
                for table in context.tables.keys():
                    context.tables[table]["status"] = "unvisited"
                return "done"

            smach.StateMachine.add(
                "RESET_TABLES",
                smach.CBState(reset_tables),
                transitions={"done": "GO_TO_TABLE"},
            )

            smach.StateMachine.add(
                "GO_TO_TABLE", GoToTable(context), transitions={"done": "serve"}
            )
