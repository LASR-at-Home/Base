import rospy
import smach
import smach_ros

# from help_me_carry import follow_person, go_to_bag, pick_up_bag
from help_me_carry import GoToBag, BagPickAndPlace
from lasr_skills import FollowPerson


class CarryMyLuggage(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])
        with self:
            smach.StateMachine.add(
                "FOLLOW_PERSON",
                FollowPerson(),
                transitions={"succeeded": "succeeded", "failed": "succeeded"},
            )

            # smach.StateMachine.add(
            #     "GO_TO_BAG",
            #     GoToBag(),
            #     transitions={"succeeded": "PICK_UP_BAG", "failed": "failed"},
            # )

            # smach.StateMachine.add(
            #     "PICK_UP_BAG",
            #     BagPickAndPlace(),
            #     transitions={"succeeded": "succeeded", "failed": "failed"},
            # )


def main():
    # Initialize ROS node
    rospy.init_node("carry_my_luggage_sm")

    # Create the state machine
    sm = CarryMyLuggage()

    # Execute the state machine
    rospy.loginfo("Starting HelpMeCarry state machine...")
    outcome = sm.execute()
    rospy.loginfo(f"State machine finished with outcome: {outcome}")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("State machine execution interrupted")
