import rospy
import smach
import smach_ros
from help_me_carry import GoToBag, BagPickAndPlace, GoToBagFailed
from lasr_skills import FollowPerson, PlayMotion, GoToLocation, Say


class CarryMyLuggage(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])
        with self:
            smach.StateMachine.add(
                "SAY_START",
                Say(text="Start of help me carry task."),
                transitions={
                    "succeeded": "PRE_FOLLOW",
                    "aborted": "PRE_FOLLOW",
                    "preempted": "PRE_FOLLOW",
                },
            )

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
                FollowPerson(fallback=True),
                transitions={"succeeded": "POST_FOLLOW", "failed": "POST_FOLLOW"},
            )

            # smach.StateMachine.add(
            #     "FOLLOW_PERSON",
            #     FollowPerson(object_avoidance=True, fallback=True),
            #     transitions={"succeeded": "POST_FOLLOW", "failed": "POST_FOLLOW"},
            # )

            smach.StateMachine.add(
                f"POST_FOLLOW",
                PlayMotion(motion_name="following_post_navigation"),
                transitions={
                    "succeeded": "SAY_GO_TO_BAG",
                    "preempted": "SAY_GO_TO_BAG",
                    "aborted": "SAY_GO_TO_BAG",
                },
            )

            smach.StateMachine.add(
                "SAY_GO_TO_BAG",
                Say(text="Please point me the bag and I will pick it up for you."),
                transitions={
                    "succeeded": "GO_TO_BAG",
                    "aborted": "GO_TO_BAG",
                    "preempted": "GO_TO_BAG",
                },
            )

            smach.StateMachine.add(
                "GO_TO_BAG",
                GoToBag(),
                transitions={
                    "succeeded": "SAY_PICK_UP_BAG",
                    "failed": "SAY_GO_TO_BAG_FAILED",
                },
            )

            smach.StateMachine.add(
                "SAY_GO_TO_BAG_FAILED",
                Say(text="Please step away I will reach my arm out."),
                transitions={
                    "succeeded": "GO_TO_BAG_FAILED",
                    "aborted": "GO_TO_BAG_FAILED",
                    "preempted": "GO_TO_BAG_FAILED",
                },
            )

            smach.StateMachine.add(
                "GO_TO_BAG_FAILED",
                GoToBagFailed(),
                transitions={
                    "succeeded": "SAY_GOING_BACK",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_PICK_UP_BAG",
                Say(text="Picking up the bag."),
                transitions={
                    "succeeded": "PICK_UP_BAG",
                    "aborted": "PICK_UP_BAG",
                    "preempted": "PICK_UP_BAG",
                },
            )

            smach.StateMachine.add(
                "PICK_UP_BAG",
                BagPickAndPlace(),
                transitions={
                    "succeeded": "SAY_GOING_BACK",
                    "failed": "SAY_GOING_BACK",
                },
            )

            smach.StateMachine.add(
                "SAY_GOING_BACK",
                Say(text="Going back to start point."),
                transitions={
                    "succeeded": "GO_BACK_TO_START_POINT",
                    "aborted": "GO_BACK_TO_START_POINT",
                    "preempted": "GO_BACK_TO_START_POINT",
                },
            )

            smach.StateMachine.add(
                f"GO_BACK_TO_START_POINT",
                GoToLocation(),
                transitions={
                    "succeeded": "SAY_END",
                    "failed": "SAY_END",
                },
            )

            smach.StateMachine.add(
                "SAY_END",
                Say(text="End of help me carry task."),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "succeeded",
                    "preempted": "succeeded",
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
