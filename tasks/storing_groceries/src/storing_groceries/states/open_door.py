

def main():
    rospy.init_node("lang_sam_state_machine")

    sm = smach.StateMachine(outcomes=["DONE", "FAILED"])
    with sm:
        smach.StateMachine.add(
            "SEGMENT_DOOR",
            LangSamState(prompt="a door"),
            transitions={"succeeded": "DONE", "failed": "FAILED"}
        )

    outcome = sm.execute()
    rospy.loginfo(f"State machine outcome: {outcome}")

if __name__ == "__main__":
    main()


