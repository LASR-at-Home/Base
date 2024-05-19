#!/usr/bin/env python3
import smach
import rospy
import os
import rospkg

DATASET_ROOT = os.path.join(
    rospkg.RosPack().get_path("lasr_vision_deepface"), "datasets"
)


class CheckKnownPeople(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["task_name"],
            output_keys=["known_people"],
        )

    def execute(self, userdata):
        try:
            dataset_path = os.path.join(DATASET_ROOT, userdata.task_name)
            print(dataset_path)
            known_people_names = [
                f
                for f in os.listdir(dataset_path)
                if os.path.isdir(os.path.join(dataset_path, f))
            ]
            rospy.set_param("/known_people", known_people_names)
            userdata.known_people = known_people_names
            return "succeeded"
        except Exception as e:
            rospy.logerr(f"Failed to get known people: {str(e)}")
            return "failed"


if __name__ == "__main__":
    rospy.init_node("check_known_people")
    sm = smach.StateMachine(outcomes=["succeeded", "failed"])
    ud = smach.UserData()
    ud.task_name = "receptionist"
    sm.userdata["task_name"] = ud.task_name
    with sm:
        smach.StateMachine.add(
            "CHECK_KNOWN_PEOPLE",
            CheckKnownPeople(),
            transitions={
                "succeeded": "succeeded",
                "failed": "failed",
            },
        )
    sm.execute()
