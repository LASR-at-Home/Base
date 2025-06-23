import rospy
import smach
import smach_ros

# from help_me_carry import follow_person, go_to_bag, pick_up_bag
from help_me_carry import GoToBag, BagPickAndPlace
from lasr_skills import FollowPerson, PlayMotion


class CarryMyLuggage(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])
        with self:
            smach.StateMachine.add(
                f"PRE_FOLLOW",
                PlayMotion(motion_name="pre_navigation"),
                transitions={
                    "succeeded": f"FOLLOW_PERSON",
                    "preempted": "FOLLOW_PERSON",
                    "aborted": "FOLLOW_PERSON",
                },
            )

            smach.StateMachine.add(
                "FOLLOW_PERSON",
                FollowPerson(),
                transitions={"succeeded": "POST_FOLLOW", "failed": "POST_FOLLOW"},
            )

            smach.StateMachine.add(
                f"POST_FOLLOW",
                PlayMotion(motion_name="following_post_navigation"),
                transitions={
                    "succeeded": f"GO_TO_BAG",
                    "preempted": "GO_TO_BAG",
                    "aborted": "GO_TO_BAG",
                },
            )

            smach.StateMachine.add(
                "GO_TO_BAG",
                GoToBag(),
                transitions={
                    "succeeded": "PICK_UP_BAG",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "PICK_UP_BAG",
                BagPickAndPlace(),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )


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
