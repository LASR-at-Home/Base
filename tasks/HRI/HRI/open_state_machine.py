import rclpy
import smach
from smach_ros import RosState

from lasr_skills.start_door_sm import StartDoorSM
from lasr_skills.go_to_location import GoToLocation

from lasr_skills.greet_and_track_node import build_greet_and_track_sm
# SM1: StartDoorSM(location=startPose)

# GoToDoor (GoToLocation(doorPose))

# SM2: 

class StartSM(smach.StateMachine):
    '''
        SM1 -> gotoDoor -> SM2
    '''
    def __init__(self, node):
        super().__init__(
            outcomes=["succeeded", "failed"],
        )

        with self:
            self.add(   # SM1
                "START_SCENARIO",
                StartDoorSM(node),
                transitions={"succeeded": "GO_TO_DOOR", "failed": "failed"},
            )
            self.add(
                "GO_TO_DOOR",
                GoToLocation(
                    node,
                    location_param="door_pose",
                ),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
            # Add SM2 here
            self.add(
                "SM_2",
                build_greet_and_track_sm(node), 
                transitions={"succeeded": "succeeded", "failed": "failed"}
            )



def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node(
        "hri",
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True,
    )

    try:
        sm = StartSM(node=node)
        outcome = sm.execute()
        node.get_logger().info(f"StartSM outcome: {outcome}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#TODO: 
# Update Poses
# 1. use ReID to face guest
# 2. Greet them (eg. Hello, what is your name and favourite drink)
# 3. Update Get getNameAndDrink/getNameOrDrink to not use predetermined list of drinks and names