import rospy
import smach
import smach_ros
from .states import Start, GoToTable, CheckTable

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion


class Phase1(smach.StateMachine):

    class GoIdle(smach.StataMachine):

        def __init__(self):

            smach.StateMachine.__init__(self, outcomes=["done"])

            idle_location = rospy.get_param("idle/pose")
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
                    rospy.sleep(rospy.Duration(30.0))
                    return "succeeded"

                smach.StateMachine.add(
                    "IDLE",
                    smach.CBState(idle_cb),
                    transitions={"succeeded": "done"},
                )

    def __init__(self, context):
        smach.StateMachine.__init__(self, outcomes=["done"])

        with self:
            smach.StateMachine.add(
                "GO_IDLE", self.GoIdle(), transitions={"done": "GO_TO_TABLE"}
            )
            smach.StateMachine.add(
                "GO_TO_TABLE", GoToTable(context), transitions={"done": "CHECK_TABLE"}
            )
            smach.StateMachine.add(
                "CHECK_TABLE",
                CheckTable(context),
                transitions={
                    "not_finished": "GO_TO_TABLE",
                    "has_free_tables": "done",
                    "no_free_tables": "GO_IDLE",
                },
            )
