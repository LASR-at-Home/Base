import smach
from smach_ros import SimpleActionState

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


from navigation_helpers import plan_to_radius

from geometry_msgs.msg import PoseStamped, Header


class GoToPerson(smach.StateMachine):

    class PlanToPerson(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["location"],
                output_keys=["target_location"],
            )

        def execute(self, userdata):
            points = plan_to_radius(userdata.location, 0.5)
            if not points:
                return "failed"
            userdata.target_location = points[0]

    def __init__(self):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["location"],
        )

        with self:

            smach.StateMachine.add(
                "PLAN_TO_PERSON",
                self.PlanToPerson(),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )

            smach.StateMachine.add(
                "GO_TO_LOCATION",
                SimpleActionState(
                    "move_base",
                    MoveBaseAction,
                    goal_cb=lambda ud, _: MoveBaseGoal(
                        target_pose=PoseStamped(
                            pose=ud.target_location, header=Header(frame_id="map")
                        )
                    ),
                    input_keys=["target_location"],
                ),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                },
            )
