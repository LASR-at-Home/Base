import rospy
import smach
import smach_ros
from lasr_skills import (
    follow_person,
    go_to_bag,
    pick_up_bag,
)


class CarryMyLuggage(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])
        with self:
            smach.StateMachine.add(
                "FOLLOW_PERSON",
                follow_person.FollowPerson(),
                transitions={
                    "succeeded": "GO_TO_BAG",
                    "failed": "GO_TO_BAG",
                },
            )

            smach.StateMachine.add(
                "GO_TO_BAG",
                go_to_bag.GoToBag(),
                transitions={
                    "succeeded": "PICK_UP_BAG",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "PICK_UP_BAG",
                pick_up_bag.BagPickAndPlace(),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )


def main():
    # Initialize ROS node
    rospy.init_node('carry_my_luggage_sm')

    # Create the state machine
    sm = CarryMyLuggage()

    # Execute the state machine
    rospy.loginfo("Starting CarryMyLuggage state machine...")
    outcome = sm.execute()
    rospy.loginfo(f"State machine finished with outcome: {outcome}")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("State machine execution interrupted")
